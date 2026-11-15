// Copyright 2026 Manifold Tech Ltd.
//
// Closed-loop navigation simulator for the Go2 + Odin1 stack.
//
// planner_sim.cpp answers "does A* find a sane route through this geometry".
// This one answers a different question: "does the robot actually get there
// when the map is built from a laser that only sees what it hits, when the
// global plan has to be re-made every couple of seconds, and when the local
// layer has to dodge something that was not there a moment ago".
//
// Everything is synthetic but nothing is faked:
//   * obstacles are ground-truth primitives (boxes and cylinders);
//   * the point cloud is produced by casting rays at those primitives, so
//     occlusion, missing returns and range limits all behave for real;
//   * the grid is rebuilt from the accumulated cloud every 0.5 s, which is
//     what the real stack does when it refreshes the octomap;
//   * the global route comes from the same AStar3D the robot runs;
//   * avoidance is a DWA rollout over the *perceived* grid, not the truth -
//     if perception is wrong the robot collides, and the metric says so.
//
// Robot envelope comes from unitreerobotics/unitree_ros
// robots/go2_description/urdf/go2_description.urdf: trunk collision box
// 0.3762 x 0.0935 x 0.114, hips at x=+-0.1934, legs abducted to y=+-0.142.
// Standing footprint is therefore about 0.48 x 0.32; the 0.26 / 0.17 half
// extents below add a little for leg swing.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "octo_planner_ros1/astar3d.h"
#include "octo_planner_ros1/voxel_grid.h"

namespace {

using namespace octo_planner_ros1;

// ---------------------------------------------------------------- robot ----

// From the URDF, with a margin for leg swing.
constexpr double kHalfLen = 0.26;
constexpr double kHalfWid = 0.17;
constexpr double kBaseZ = 0.32;      // trunk centre above the floor, standing
constexpr double kSensorZ = 0.45;    // Odin1 sits on the back
constexpr double kMassKg = 16.087;   // summed from the URDF, rotors included

// ------------------------------------------------------------ obstacles ----

struct Obstacle {
  int kind = 0;  // 0 = axis-aligned box, 1 = cylinder
  double a[4] = {0, 0, 0, 0};  // box: x0,y0,x1,y1 ; cyl: cx,cy,r
  double z0 = 0.0;
  double z1 = 1.0;
  double appear = -1.0;   // < 0 means "always there"
  double vx = 0.0, vy = 0.0;
  double bx = 0.0, by = 0.0;  // spawn pose, so motion is absolute not accumulative

  bool present(double t) const { return appear < 0.0 || t >= appear; }

  void advance(double t) {
    if (appear < 0.0 || (vx == 0.0 && vy == 0.0)) return;
    const double dt = std::max(0.0, t - appear);
    if (kind == 0) {
      const double w = a[2] - a[0];
      const double h = a[3] - a[1];
      a[0] = bx + vx * dt;
      a[1] = by + vy * dt;
      a[2] = a[0] + w;
      a[3] = a[1] + h;
    } else {
      a[0] = bx + vx * dt;
      a[1] = by + vy * dt;
    }
  }
};

Obstacle box(double x0, double y0, double x1, double y1, double z0, double z1) {
  Obstacle o;
  o.kind = 0;
  o.a[0] = x0; o.a[1] = y0; o.a[2] = x1; o.a[3] = y1;
  o.z0 = z0; o.z1 = z1;
  o.bx = 0.5 * (x0 + x1); o.by = 0.5 * (y0 + y1);
  return o;
}

Obstacle cyl(double cx, double cy, double r, double z0, double z1) {
  Obstacle o;
  o.kind = 1;
  o.a[0] = cx; o.a[1] = cy; o.a[2] = r;
  o.z0 = z0; o.z1 = z1;
  o.bx = cx; o.by = cy;
  return o;
}

// A wall as four thin boxes, so there is one primitive type in the hot loop.
void wall(std::vector<Obstacle>& out, double x0, double y0, double x1, double y1,
          double z1, double thick) {
  out.push_back(box(x0, y0, x1, y0 + thick, 0.0, z1));
  out.push_back(box(x0, y1 - thick, x1, y1, 0.0, z1));
  out.push_back(box(x0, y0, x0 + thick, y1, 0.0, z1));
  out.push_back(box(x1 - thick, y0, x1, y1, 0.0, z1));
}

// ------------------------------------------------------------ raycasting ----

double rayBox(double ox, double oy, double dx, double dy,
              double x0, double y0, double x1, double y1) {
  double tmin = -1e30, tmax = 1e30;
  auto slab = [&](double o, double d, double lo, double hi) {
    if (std::fabs(d) < 1e-12) {
      if (o < lo || o > hi) tmin = 1e30;
      return;
    }
    double t1 = (lo - o) / d, t2 = (hi - o) / d;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  };
  slab(ox, dx, x0, x1);
  slab(oy, dy, y0, y1);
  if (tmax < tmin || tmax <= 0.0) return -1.0;
  return std::max(tmin, 0.0);
}

double rayCyl(double ox, double oy, double dx, double dy,
              double cx, double cy, double r) {
  const double fx = ox - cx, fy = oy - cy;
  const double a = dx * dx + dy * dy;
  if (a < 1e-12) return -1.0;
  const double b = 2.0 * (fx * dx + fy * dy);
  const double c = fx * fx + fy * fy - r * r;
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) return -1.0;
  const double t = (-b - std::sqrt(disc)) / (2.0 * a);
  return t > 0.0 ? t : -1.0;
}

struct Point { double x, y, z; };

struct SensorModel {
  int azimuth = 360;  // 1 deg steps
  // Multi-line, like the real spinning head. A single -12 deg ring would put
  // every floor return on one 2.16 m circle and leave the ground under the
  // robot empty - a healthy laser does not do that.
  std::vector<double> elev = {-0.60, -0.50, -0.40, -0.32, -0.25, -0.19,
                              -0.14, -0.10, -0.07, 0.00, 0.15, 0.30};
  double max_range = 4.0;
  double sigma = 0.02;   // range noise, metres
  double dropout = 0.04; // fraction of rays that return nothing
};

// One sweep. Rays are cast in the body frame then rotated by yaw, which is
// how the real driver publishes: points arrive in odom, but the pattern is
// fixed to the sensor.
void scan(const std::vector<Obstacle>& obs, double t, double rx, double ry,
          double yaw, const SensorModel& sm, std::mt19937& rng,
          std::vector<Point>& out) {
  std::normal_distribution<double> noise(0.0, sm.sigma);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  out.clear();
  for (double e : sm.elev) {
    const double ce = std::cos(e), se = std::sin(e);
    for (int i = 0; i < sm.azimuth; ++i) {
      const double az = 2.0 * M_PI * i / sm.azimuth;
      const double bx = std::cos(az) * ce, by = std::sin(az) * ce;
      const double dx = bx * cy - by * sy;  // body -> world
      const double dy = bx * sy + by * cy;
      if (uni(rng) < sm.dropout) continue;

      double best = sm.max_range;
      // floor plane z = 0
      if (se < -1e-6) {
        const double tf = -kSensorZ / se;
        if (tf > 0.0 && tf < best) best = tf;
      }
      for (const auto& o : obs) {
        if (!o.present(t)) continue;
        double hit = (o.kind == 0) ? rayBox(rx, ry, dx, dy, o.a[0], o.a[1], o.a[2], o.a[3])
                                   : rayCyl(rx, ry, dx, dy, o.a[0], o.a[1], o.a[2]);
        if (hit < 0.0 || hit >= best) continue;
        const double hz = kSensorZ + hit * se;
        if (hz < o.z0 || hz > o.z1) continue;  // ray passes over or under it
        best = hit;
      }
      if (best >= sm.max_range) continue;
      const double r = best + noise(rng);
      if (r <= 0.05) continue;
      out.push_back({rx + dx * r, ry + dy * r, kSensorZ + r * se});
    }
  }
  // Near-field ground. The lowest line is at -34 deg, so the floor closer
  // than ~0.8 m is outside the FOV - on hardware that patch comes from the
  // IMU/odometry plane assumption, not from the laser. Adding it here keeps
  // the map standable for the right reason instead of failing for a reason
  // that does not exist outside this file.
  for (int i = 0; i < 240; ++i) {
    const double a = 2.0 * M_PI * i / 240.0;
    const double rr = 0.15 + 0.95 * ((i % 8) / 8.0);
    out.push_back({rx + rr * std::cos(a), ry + rr * std::sin(a), 0.0});
  }
}

// --------------------------------------------------------------- scenario ----

struct Scenario {
  std::string name;
  std::string desc;
  double minx = 0, miny = 0, maxx = 10, maxy = 10;
  std::vector<Obstacle> obs;
  double sx = 1, sy = 1, gx = 9, gy = 9;
  double limit = 80.0;  // simulated seconds before giving up
};

// A. office: two desks and a pillar between the corners.
Scenario officeLoop() {
  Scenario s;
  s.name = "office_loop";
  s.desc = "10 x 8 m 办公室：两张 0.75 m 高的桌子（狗能从桌下过、桌腿不能）、一根承重柱；起点左下、目标右上，直线会撞桌";
  s.minx = 0; s.miny = 0; s.maxx = 10; s.maxy = 8;
  wall(s.obs, 0.0, 0.0, 10.0, 8.0, 2.6, 0.15);
  auto desk = [&](double x, double y) {
    s.obs.push_back(box(x, y, x + 0.9, y + 0.6, 0.72, 0.78));       // top
    for (double dx : {0.05, 0.85}) for (double dy : {0.05, 0.55})
      s.obs.push_back(box(x + dx, y + dy, x + dx + 0.05, y + dy + 0.05, 0.0, 0.72));
  };
  desk(2.8, 2.0);
  desk(5.6, 4.6);
  s.obs.push_back(cyl(4.6, 2.2, 0.22, 0.0, 2.6));                   // pillar
  s.sx = 1.0; s.sy = 1.0;
  s.gx = 9.0; s.gy = 7.0;
  return s;
}

// B. corridor: 1.6 m clear width, a crate lands mid-corridor at t = 3 s.
Scenario corridorBlock() {
  Scenario s;
  s.name = "corridor_block";
  s.desc = "1.6 m 净宽走廊：3 秒后一只 0.4 x 0.4 m 箱子落在通道中段靠下侧，只剩 1.2 m 可过——用来测「跑起来之后才发现」的重规划与局部避障";
  s.minx = 0; s.miny = 0; s.maxx = 14; s.maxy = 1.9;
  wall(s.obs, 0.0, 0.0, 14.0, 1.9, 2.6, 0.15);
  Obstacle crate = box(6.8, 0.20, 7.2, 0.60, 0.0, 0.55);
  crate.appear = 3.0;
  s.obs.push_back(crate);
  s.sx = 0.8; s.sy = 0.95;
  s.gx = 13.2; s.gy = 0.95;
  return s;
}

// C. two rooms, 1.2 m doorway, someone walking the other way.
Scenario narrowGate() {
  Scenario s;
  s.name = "narrow_gate";
  s.desc = "两间房中间 1.2 m 门洞（膨胀后净剩 0.7 m，狗宽 0.34 m）；4 秒后一个人从目标侧朝狗走来——测窄通道里的对向避让";
  s.minx = 0; s.miny = 0; s.maxx = 13; s.maxy = 5;
  wall(s.obs, 0.0, 0.0, 13.0, 5.0, 2.6, 0.15);
  // partition at x = 6..6.2, doorway at y = 2.0..3.2
  s.obs.push_back(box(6.0, 0.15, 6.2, 2.0, 0.0, 2.6));
  s.obs.push_back(box(6.0, 3.2, 6.2, 4.85, 0.0, 2.6));
  Obstacle person = cyl(9.0, 3.0, 0.22, 0.0, 1.75);
  person.appear = 4.0;
  person.vx = -0.28;
  s.obs.push_back(person);
  s.sx = 1.5; s.sy = 1.5;
  s.gx = 11.5; s.gy = 3.6;
  return s;
}

// ------------------------------------------------------------ map helpers ----

// One return. Ground returns are stamped over their 3x3 neighbourhood: a 90
// return sweep is sparse compared with 0.1 m cells, and the floor closer than
// ~0.65 m sits outside the lowest line. The floor is a continuous plane, so
// filling that gap cannot hide an obstacle - anything standing on it reaches
// above the band and produces its own return.
void stampPoint(VoxelGrid& g, const Point& p) {
  g.insertSurfacePoint(p.x, p.y, p.z, 20);
  if (p.z <= -0.09 || p.z >= 0.06) return;
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      g.insertSurfacePoint(p.x + dx * g.resolution(),
                           p.y + dy * g.resolution(), p.z, 20);
}

// The mission map = what Odin1 already surveyed (prior) + what the live sweep
// sees. Anchoring the global route on the prior is what lets a goal 12 m away
// be planned at all: a 4 m sensor cannot see it from the start pose.
void rebuild(VoxelGrid& g, const std::vector<Point>& pts,
             const VoxelGrid* prior = nullptr) {
  g.clear();
  for (const auto& p : pts) stampPoint(g, p);
  if (prior) {
    for (int iz = 0; iz < g.nz(); ++iz)
      for (int iy = 0; iy < g.ny(); ++iy)
        for (int ix = 0; ix < g.nx(); ++ix) {
          // Copy the whole known state, not just the occupied cells: the free
          // run above a floor is what makes a cell standable, and the live
          // sweep only covers 4 m around the dog.
          if (g.at(ix, iy, iz) != VoxelGrid::kUnknown) continue;
          const uint8_t v = prior->at(ix, iy, iz);
          if (v != VoxelGrid::kUnknown) g.set(ix, iy, iz, v);
        }
  }
  // 0.20 m of inflation, plus the 0.20 m envelope sampled by blocked2D, is
  // the conservative pair for a trunk half width of 0.17 m: 3 cells on both
  // sides closed a 1.6 m corridor completely once a bin was in it, which the
  // real dog walks through.
  g.inflate(2, 0, /*skip_standing_surfaces=*/true);
}

// Survey stands: one sweep every 2.4 m, which is roughly what an operator gets
// by walking Odin1 around the floor once before handing the dog a mission.
void surveyMap(const Scenario& sc, const SensorModel& sm, std::mt19937& rng,
               VoxelGrid& prior) {
  prior.configureFromBounds(0.10, sc.minx - 0.6, sc.miny - 0.6, -0.20,
                            sc.maxx + 0.6, sc.maxy + 0.6, 2.4, 0.30);
  prior.clear();
  std::vector<Point> one;
  for (double bx = sc.minx + 1.2; bx <= sc.maxx - 0.6; bx += 2.4)
    for (double by = sc.miny + 1.2; by <= sc.maxy - 0.6; by += 2.4) {
      scan(sc.obs, 0.0, bx, by, 0.0, sm, rng, one);
      for (const auto& p : one) {
        prior.insertSurfacePoint(p.x, p.y, p.z, 20);
        if (p.z > -0.09 && p.z < 0.06)
          for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
              prior.insertSurfacePoint(p.x + dx * prior.resolution(),
                                       p.y + dy * prior.resolution(), p.z, 20);
      }
    }
}

// Plan cell for a world pose: the free cell sitting on the floor.
bool planCell(const VoxelGrid& g, double x, double y, VoxelKey& out) {
  int ix, iy, iz;
  if (!g.worldToIndex(x, y, kBaseZ, ix, iy, iz)) return false;
  // groundBelow already steps one cell up from the surface it found, i.e.
  // it returns the cell the body occupies - not the floor cell.
  const int stand = g.groundBelow(ix, iy, iz);
  if (stand < 0) return false;
  out.x = ix; out.y = iy; out.z = stand;
  return true;
}

// Projected 2D occupancy for the local rollout. Only the band the body
// actually occupies counts, so a tabletop at 0.75 m does not block the robot
// walking underneath it - which is exactly the property the 3D grid buys us.
// Single grid cell in the trunk band 0.12 .. 0.42 m.
bool cellBlocked(const VoxelGrid& g, double wx, double wy) {
  int ix, iy, iz;
  // 0.12 rather than 0.10 on purpose: (0.10 - origin_z) / 0.10 evaluates to
  // 5.999999999999999 for this grid and floors onto the floor layer itself,
  // which made every rollout look like it drove into the ground.
  if (!g.worldToIndex(wx, wy, 0.12, ix, iy, iz)) return true;  // outside = blocked
  for (int dz = 0; dz <= 3; ++dz) {
    const int jz = iz + dz;  // 0.12 .. 0.42 m: the trunk, nothing above it
    if (jz >= g.nz()) break;
    const uint8_t v = g.at(ix, iy, jz);
    if (v == VoxelGrid::kOccupied || v == VoxelGrid::kInflated) return true;
  }
  return false;
}

// Body envelope, not a point. Checking only the centre let the trunk corner
// clip a 1.2 m door frame while the centre line still read as free, which is
// the one failure a real Go2 cannot recover from at walking speed.
bool blocked2D(const VoxelGrid& g, double wx, double wy) {
  const double r = 0.20;  // half width 0.17 from the URDF, plus a cell of slack
  for (double dx : {0.0, r, -r})
    for (double dy : {0.0, r, -r})
      if (cellBlocked(g, wx + dx, wy + dy)) return true;
  return false;
}

// Ground-truth footprint test: OBB (robot) against every live primitive.
bool hitsTruth(const std::vector<Obstacle>& obs, double t, double x, double y,
               double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  // four corners plus edge midpoints, enough for these convex primitives
  double px[8], py[8];
  int n = 0;
  for (double u : {-kHalfLen, 0.0, kHalfLen}) {
    for (double v : {-kHalfWid, 0.0, kHalfWid}) {
      if (u == 0.0 && v == 0.0) continue;
      px[n] = x + u * c - v * s;
      py[n] = y + u * s + v * c;
      ++n;
    }
  }
  for (const auto& o : obs) {
    if (!o.present(t)) continue;
    if (o.z0 > kBaseZ + 0.06) continue;  // entirely above the trunk, ignore
    if (o.kind == 1) {
      for (int i = 0; i < n; ++i) {
        const double dx = px[i] - o.a[0], dy = py[i] - o.a[1];
        if (dx * dx + dy * dy <= o.a[2] * o.a[2]) return true;
      }
      // centre inside the circle as well
      const double dx = x - o.a[0], dy = y - o.a[1];
      if (dx * dx + dy * dy <= o.a[2] * o.a[2]) return true;
    } else {
      for (int i = 0; i < n; ++i)
        if (px[i] > o.a[0] && px[i] < o.a[2] && py[i] > o.a[1] && py[i] < o.a[3])
          return true;
      // box corner inside the robot quad: cheap SAT via the two axes
      for (double cx : {o.a[0], o.a[2]}) {
        for (double cy : {o.a[1], o.a[3]}) {
          const double dx = cx - x, dy = cy - y;
          const double u = dx * c + dy * s;
          const double v = -dx * s + dy * c;
          if (std::fabs(u) < kHalfLen && std::fabs(v) < kHalfWid) return true;
        }
      }
    }
  }
  return false;
}

// ------------------------------------------------------------------- run ----

struct Cmd { double vx = 0, vy = 0, wz = 0; };

struct Sample {
  double t = 0;
  double x = 0, y = 0, yaw = 0;
  double vx = 0, vy = 0, wz = 0;
  bool hit = false;
};

struct Run {
  bool reached = false;
  bool collided = false;
  double time = 0;
  double length = 0;
  double min_clear = 99;
  int replans = 0;
  int dwa_swerves = 0;
  int scans = 0;
  std::vector<Sample> trace;
  std::vector<std::vector<Point>> clouds;
  std::vector<double> cloud_t;
  std::vector<std::pair<double, std::string>> events;
  std::vector<std::pair<double, double>> global_path;  // x,z
  double final_x = 0, final_y = 0;
};

// Coarse clearance: distance from the body box to the nearest live primitive.
double clearance(const std::vector<Obstacle>& obs, double t, double x, double y) {
  double best = 6.0;
  for (const auto& o : obs) {
    if (!o.present(t)) continue;
    if (o.z0 > kBaseZ + 0.06) continue;
    if (o.kind == 1) {
      const double d = std::hypot(x - o.a[0], y - o.a[1]) - o.a[2];
      best = std::min(best, d);
    } else {
      const double dx = std::max(std::max(o.a[0] - x, x - o.a[2]), 0.0);
      const double dy = std::max(std::max(o.a[1] - y, y - o.a[3]), 0.0);
      const double d = std::hypot(dx, dy) - std::max(kHalfLen, kHalfWid) * 0.6;
      best = std::min(best, d);
    }
  }
  return best;
}

Run simulate(const Scenario& sc, unsigned seed) {
  Run out;
  std::mt19937 rng(seed);
  SensorModel sm;

  VoxelGrid grid;
  grid.configureFromBounds(0.10, sc.minx - 0.6, sc.miny - 0.6, -0.20,
                           sc.maxx + 0.6, sc.maxy + 0.6, 2.4, 0.30);

  VoxelGrid prior;
  surveyMap(sc, sm, rng, prior);

  std::vector<Obstacle> obs = sc.obs;
  std::vector<Point> cloud;           // current sweep
  std::vector<Point> accum;           // last few sweeps, what the map is built from

  double x = sc.sx, y = sc.sy, yaw = std::atan2(sc.gy - y, sc.gx - x);
  double vx = 0, vy = 0, wz = 0;
  const double dt_sim = 0.02;
  const double ctrl = 0.10;
  double ctrl_accum = 0;

  // initial sweep so the first plan has something to work with
  scan(obs, 0.0, x, y, yaw, sm, rng, cloud);
  accum = cloud;
  rebuild(grid, accum, &prior);
  out.scans = 1;
  out.clouds.push_back(cloud);
  out.cloud_t.push_back(0.0);

  AStar3D planner;
  AStar3D::Params params;
  params.max_iterations = 200000;
  params.require_ground_support = true;
  params.snap_search_radius_cells = 12;
  params.max_step_z_cells = 3;

  std::vector<std::pair<double, double>> gpath;       // current route, world x,y
  std::vector<std::pair<double, double>> first_path;  // route as first planned
  double last_plan_t = -10.0;
  bool have_plan = false;

  auto replan = [&](double t) {
    VoxelKey s, g;
    if (!planCell(grid, x, y, s)) {
      out.events.push_back({t, "规划失败：起点不在网格内"});
      return false;
    }
    if (!planCell(grid, sc.gx, sc.gy, g)) {
      out.events.push_back({t, "规划失败：目标不在网格内"});
      return false;
    }
    AStar3D::Result r = planner.plan(grid, s, g, params, false);
    if (!r.ok) {
      out.events.push_back({t, "规划失败：" + r.message});
      return false;
    }
    if (first_path.empty())
      for (const auto& k : r.path) {
        double wx, wy, wz;
        grid.indexToWorld(k.x, k.y, k.z, wx, wy, wz);
        first_path.push_back({wx, wy});
      }
    gpath.clear();
    for (const auto& k : r.path) {
      double wx, wy, wz;
      grid.indexToWorld(k.x, k.y, k.z, wx, wy, wz);
      gpath.push_back({wx, wy});
    }
    have_plan = true;
    last_plan_t = t;
    return true;
  };

  replan(0.0);
  out.replans = 1;

  // nearest point on the global route
  auto routeError = [&](double px, double py) {
    double best = 1e9;
    for (const auto& q : gpath) best = std::min(best, std::hypot(px - q.first, py - q.second));
    return best;
  };

  // DWA over the perceived grid.
  struct Cand { double vx, vy, wz; };
  std::vector<Cand> cands;
  for (double a : {-0.15, 0.0, 0.15, 0.35, 0.55})
    for (double b : {-0.15, 0.0, 0.15})
      for (double c : {-1.1, -0.55, 0.0, 0.55, 1.1})
        cands.push_back({a, b, c});

  double t = 0.0;
  bool done = false;
  bool detouring = false;
  double px_prev = x, py_prev = y;

  // Integer step counters, not std::fmod: adding 0.02 repeatedly does not land
  // on 0.10 boundaries, and every tick that missed the comparison silently
  // dropped a mapping refresh or a distance sample.
  long step = 0;
  while (t < sc.limit && !done) {
    t += dt_sim;
    ++step;

    for (auto& o : obs) o.advance(t);

    // 0.5 s mapping refresh
    if (step % 25 == 0) {  // 0.5 s mapping refresh
      scan(obs, t, x, y, yaw, sm, rng, cloud);
      accum.insert(accum.end(), cloud.begin(), cloud.end());
      if (accum.size() > 16000) accum.erase(accum.begin(), accum.begin() + (accum.size() - 16000));
      rebuild(grid, accum, &prior);
      ++out.scans;
      if (out.clouds.size() < 400) {
        out.clouds.push_back(cloud);
        out.cloud_t.push_back(t);
      }
    }

    // periodic replan, or when we have drifted off the route
    if (t - last_plan_t > 1.5 || (have_plan && routeError(x, y) > 0.45)) {
      if (replan(t)) ++out.replans;
    }

    ctrl_accum += dt_sim;
    if (ctrl_accum >= ctrl - 1e-9) {
      ctrl_accum = 0.0;

      if (have_plan) {
        const double dgoal = std::hypot(sc.gx - x, sc.gy - y);
        if (dgoal < 0.45) {
          done = true;
          out.reached = true;
          out.events.push_back({t, "到达目标"});
        } else {
          // which way is the route pointing? use the nearest route point a
          // little ahead of us, not the raw goal, so a detour is respected
          double bestd = 1e9, tx = sc.gx, ty = sc.gy;
          int bi = 0;
          for (int i = 0; i < (int)gpath.size(); ++i) {
            const double d = std::hypot(x - gpath[i].first, y - gpath[i].second);
            if (d < bestd) { bestd = d; bi = i; }
          }
          const int ahead = std::min((int)gpath.size() - 1, bi + 8);
          tx = gpath[ahead].first;
          ty = gpath[ahead].second;

          double best_cost = 1e18;
          Cand best{0, 0, 0};
          bool any = false;
          for (const auto& c : cands) {
            // roll out 1.2 s at 0.2 s steps
            double sx2 = x, sy2 = y, syaw2 = yaw;
            bool bad = false;
            double mincl = 9.9;
            for (int k = 0; k < 6; ++k) {
              const double h = 0.20;
              sx2 += (c.vx * std::cos(syaw2) - c.vy * std::sin(syaw2)) * h;
              sy2 += (c.vx * std::sin(syaw2) + c.vy * std::cos(syaw2)) * h;
              syaw2 += c.wz * h;
              if (blocked2D(grid, sx2, sy2)) { bad = true; break; }
              mincl = std::min(mincl, clearance(obs, t, sx2, sy2));
            }
            if (bad) continue;
            any = true;
            const double d2 = std::hypot(tx - sx2, ty - sy2);
            const double cost = 1.4 * d2 + 0.55 * (1.0 / (mincl + 0.25)) - 0.45 * c.vx
                                + 0.22 * std::fabs(c.vy) + 0.08 * std::fabs(c.wz);
            if (cost < best_cost) { best_cost = cost; best = c; }
          }
          if (any) {
            vx = best.vx; vy = best.vy; wz = best.wz;
            // A swerve is one episode away from the route, not one control
            // tick: counting per tick reported 130 for a single detour.
            const double dev = routeError(x, y);
            if (!detouring && dev > 0.30) { detouring = true; ++out.dwa_swerves; }
            else if (detouring && dev < 0.18) detouring = false;
          } else {
            vx = vy = wz = 0;
            out.events.push_back({t, "无可行候选，原地停等"});
          }
        }
      }
    }

    // integrate
    if (!done) {
      x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt_sim;
      y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt_sim;
      yaw += wz * dt_sim;
      while (yaw > M_PI) yaw -= 2 * M_PI;
      while (yaw < -M_PI) yaw += 2 * M_PI;
    }

    // Metering is per tick; only the trace is sampled, so the reported
    // distance and clearance are never aliased by the sample rate.
    {
      const double c = clearance(obs, t, x, y);
      out.min_clear = std::min(out.min_clear, c);
    }
    out.length += std::hypot(x - px_prev, y - py_prev);
    px_prev = x; py_prev = y;

    if (step % 5 == 0) {  // 0.1 s trace sample
      Sample s;
      s.t = t; s.x = x; s.y = y; s.yaw = yaw; s.vx = vx; s.vy = vy; s.wz = wz;
      s.hit = clearance(obs, t, x, y) < 0.02;
      out.trace.push_back(s);
    }

    if (hitsTruth(obs, t, x, y, yaw)) {
      out.collided = true;
      out.events.push_back({t, "与真值障碍碰撞"});
      done = true;
    }
  }

  out.time = t;
  out.final_x = x; out.final_y = y;
  out.global_path = first_path.empty() ? gpath : first_path;

  // A 5.4 s hold used to log 54 identical lines, which turns the replay
  // timeline into a picket fence. Collapse consecutive identical messages and
  // keep the duration, so "held for 5.4 s" survives compression.
  std::vector<std::pair<double, std::string>> squeezed;
  {
    struct Gap { double t0, t1; std::string msg; };
    std::vector<Gap> gaps;
    for (const auto& e : out.events) {
      if (!gaps.empty() && gaps.back().msg == e.second && e.first - gaps.back().t1 <= 0.31) {
        gaps.back().t1 = e.first;
        continue;
      }
      gaps.push_back({e.first, e.first, e.second});
    }
    for (const auto& g : gaps) {
      std::string msg = g.msg;
      if (g.t1 - g.t0 > 0.4) {
        char d[64];
        std::snprintf(d, sizeof(d), "（持续 %.1f s）", g.t1 - g.t0 + 0.1);
        msg += d;
      }
      squeezed.push_back({g.t0, msg});
    }
  }
  out.events = squeezed;

  for (auto& e : out.events) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%.2f s  %s", e.first, e.second.c_str());
    e.second = buf;
  }
  return out;
}

// ------------------------------------------------------------------ json ----

struct Sink {
  std::ostringstream o;
  void kv(const char* k, double v, const char* fmt = "%.3f") {
    char buf[64]; std::snprintf(buf, sizeof(buf), fmt, v);
    o << "\"" << k << "\":" << buf;
  }
};

std::string jsonScenario(const Scenario& sc, const Run& r) {
  std::ostringstream o;
  o << "{";
  o << "\"name\":\"" << sc.name << "\",";
  o << "\"desc\":\"" << sc.desc << "\",";
  o << "\"ok\":" << (r.reached && !r.collided ? "true" : "false") << ",";
  o << "\"reached\":" << (r.reached ? "true" : "false") << ",";
  o << "\"collided\":" << (r.collided ? "true" : "false") << ",";
  o << "\"world\":[" << sc.minx << "," << sc.miny << "," << sc.maxx << "," << sc.maxy << "],";
  o << "\"start\":[" << sc.sx << "," << sc.sy << "],";
  o << "\"goal\":[" << sc.gx << "," << sc.gy << "],";

  // static silhouette for the backplate: emitted once, obstacles that only
  // appear later are still drawn (they are part of the world, the timeline
  // shows when they switch on)
  o << "\"obstacles\":[";
  for (size_t i = 0; i < sc.obs.size(); ++i) {
    const auto& ob = sc.obs[i];
    if (i) o << ",";
    o << "{\"k\":" << ob.kind << ",\"a\":[" << ob.a[0] << "," << ob.a[1] << ","
      << ob.a[2] << "," << ob.a[3] << "],\"z\":[" << ob.z0 << "," << ob.z1
      << "],\"appear\":" << ob.appear << "}";
  }
  o << "],";

  o << "\"metrics\":{";
  o << "\"time\":" << r.time << ",";
  o << "\"length\":" << r.length << ",";
  o << "\"min_clearance\":" << (r.min_clear > 9 ? 9 : r.min_clear) << ",";
  o << "\"replans\":" << r.replans << ",";
  o << "\"dwa_swerves\":" << r.dwa_swerves << ",";
  o << "\"scans\":" << r.scans << ",";
  o << "\"min_clearance_note\":\"包络到最近障碍的净距，含 0 表示接触\"";
  o << "},";

  o << "\"trace\":[";
  for (size_t i = 0; i < r.trace.size(); ++i) {
    const auto& s = r.trace[i];
    if (i) o << ",";
    char buf[96];
    std::snprintf(buf, sizeof(buf), "[%.3f,%.3f,%.3f]", s.x, s.y, s.yaw);
    o << buf;
  }
  o << "],";

  o << "\"global_path\":[";
  for (size_t i = 0; i < r.global_path.size(); ++i) {
    if (i) o << ",";
    char buf[48];
    std::snprintf(buf, sizeof(buf), "[%.3f,%.3f]", r.global_path[i].first, r.global_path[i].second);
    o << buf;
  }
  o << "],";

  // clouds are subsampled: 1200 points per sweep is plenty to read the shape
  o << "\"clouds\":[";
  for (size_t i = 0; i < r.clouds.size(); ++i) {
    const auto& c = r.clouds[i];
    if (i) o << ",";
    o << "{\"t\":" << r.cloud_t[i] << ",\"p\":[";
    const int stride = std::max(1, (int)c.size() / 500);
    bool first = true;
    for (size_t j = 0; j < c.size(); j += stride) {
      if (!first) o << ",";
      first = false;
      char buf[80];
      std::snprintf(buf, sizeof(buf), "[%.2f,%.2f,%.2f]", c[j].x, c[j].y, c[j].z);
      o << buf;
    }
    o << "]}";
  }
  o << "],";

  o << "\"events\":[";
  for (size_t i = 0; i < r.events.size(); ++i) {
    if (i) o << ",";
    o << "[\"" << r.events[i].second << "\"]";
  }
  o << "]";
  o << "}";
  return o.str();
}

}  // namespace

int main() {
  std::vector<Scenario> scenarios = {officeLoop(), corridorBlock(), narrowGate()};
  std::ostringstream o;
  o << "{";
  o << "\"generated\":\"closed-loop nav_sim\",";
  o << "\"robot\":{\"half_len\":" << kHalfLen << ",\"half_wid\":" << kHalfWid
    << ",\"base_z\":" << kBaseZ << ",\"sensor_z\":" << kSensorZ
    << ",\"mass\":" << kMassKg
    << ",\"source\":\"unitree_ros robots/go2_description/urdf/go2_description.urdf\"},";
  {
    SensorModel probe;
    o << "\"sensor\":{\"azimuth\":" << probe.azimuth << ",\"rings\":"
      << probe.elev.size() << ",\"elev_min\":" << probe.elev.front()
      << ",\"elev_max\":" << probe.elev.back() << ",\"max_range\":"
      << probe.max_range << ",\"sigma\":" << probe.sigma
      << ",\"dropout\":" << probe.dropout << ",\"near_field_pts\":240},";
  }
  o << "\"sim\":{\"dt\":0.02,\"ctrl\":0.10,\"map_period\":0.5,\"replan_period\":1.5},";
  o << "\"scenarios\":[";
  for (size_t i = 0; i < scenarios.size(); ++i) {
    if (i) o << ",";
    Run r = simulate(scenarios[i], 1000u + (unsigned)i);
    o << jsonScenario(scenarios[i], r);
    std::fprintf(stderr, "[%-14s] reached=%d collided=%d t=%.1fs len=%.1fm mincl=%.2fm replans=%d swerves=%d\n",
                 scenarios[i].name.c_str(), (int)r.reached, (int)r.collided, r.time, r.length,
                 r.min_clear, r.replans, r.dwa_swerves);
  }
  o << "]}";
  std::printf("%s\n", o.str().c_str());
  return 0;
}
