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
#include <map>

#include <array>
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
          double base_z, double yaw, const SensorModel& sm, std::mt19937& rng,
          std::vector<Point>& out, double ztol = 0.03) {
  const double sz0 = base_z + kSensorZ;  // sensor height above the standing face
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
        const double hz = sz0 + hit * se;
        // Half-cell tolerance: a ray that lands on the top face reports a
        // height a few millimetres above z1, and a strict test threw those
        // returns away. On a staircase that meant every tread return was
        // dropped and the floor plane took over - the whole flight vanished.
        if (hz < o.z0 - ztol || hz > o.z1 + ztol) continue;
        best = hit;
      }
      if (best >= sm.max_range) continue;
      const double r = best + noise(rng);
      if (r <= 0.05) continue;
      out.push_back({rx + dx * r, ry + dy * r, sz0 + r * se});
    }
  }
  // Near-field ground. The lowest line is at -34 deg, so the floor closer
  // than ~0.8 m is outside the FOV - on hardware that patch comes from the
  // IMU/odometry plane assumption, not from the laser. Adding it here keeps
  // the map standable for the right reason instead of failing for a reason
  // that does not exist outside this file.
  // Flat ground only. This ring is stamped at the height the body stands at,
  // and on a flight that height changes every riser - so the fill ran down the
  // whole run as a solid wall. Measured at x 5.40 / y 1.30: 0.8 m of occupied
  // (z 1.60-2.35) where the true treads are three thin layers, and column tops
  // read a flat 2.80 m for five consecutive treads that should descend 2.46 ->
  // 1.70. On level floor base_z never changes and the fill is harmless, which
  // is why five shipping scenes never showed it; on a staircase the sweep hits
  // the treads directly and nothing here is needed.
  if (base_z < 0.10) {
    for (int i = 0; i < 240; ++i) {
      const double a = 2.0 * M_PI * i / 240.0;
      const double rr = 0.15 + 0.95 * ((i % 8) / 8.0);
      out.push_back({rx + rr * std::cos(a), ry + rr * std::sin(a), base_z - 0.02});
    }
  }
}

// --------------------------------------------------------------- scenario ----

struct Zone { double x0, x1, y0, y1, z0, z1; };

struct Scenario {
  std::string name;
  std::string desc;
  // Which local planner drives this run. "neupan" is the deployed one
  // (robots/go2/neupan/planner.yaml): differential drive, a 1.0 s MPC window,
  // and the obstacle field taken straight from the point set with a hard
  // clearance threshold - no occupancy grid and no inflation. "dwa" is the
  // sampling planner this simulator grew, which does use the inflated grid.
  std::string planner = "neupan";
  double minx = 0, miny = 0, maxx = 10, maxy = 10;
  std::vector<Obstacle> obs;
  double sx = 1, sy = 1, gx = 9, gy = 9;
  double sz = 0.0, gz = 0.0;          // standing height at start / goal
  double zmin = -0.20, zmax = 2.4;    // vertical extent of the grid
  std::vector<Zone> stair;            // where stair_gait takes over from DWA
  // Legs handed over one at a time (x, y, standing height). Empty means "one
  // leg straight to the goal".
  std::vector<std::array<double, 3>> via;
  std::vector<AStar3D::Params::Corridor> corridors;  // keep routes on the nose
  // Survey stands the operator walked Odin1 through before handing over the
  // mission: x, y, standing height. Empty means "one level, walk the grid".
  std::vector<std::array<double, 3>> survey_pts;
  // The stair core's prior comes from known geometry, not from the walk-through.
  // Standing on a tread puts the next tread 0.56 rad above the lowest line, and
  // the sensor stops at +0.30, so no stand can see the flight it is standing
  // on. Live deployment seeds the stair prior from the model before the first
  // mission; the live sweep still owns everything the dog sees on the way up,
  // and collisions are still checked against the truth.
  bool seed_stair_prior = false;
  // Two-storey job: the hand-off covers the flights only. The landing stays
  // under the local planner, which can see the return flight's balustrade and
  // turn into it; stair_gait drives blind and would cut that corner.
  bool flights_only = false;
  // Echo returns land a few millimetres above the tread's top face; rejecting
  // them deletes the whole stair from the map (the floor plane then stands in
  // for it). Only the stair scenes need the slack.
  double ztol = 0.03;
  double limit = 80.0;  // simulated seconds before giving up
};

bool inStair(const Scenario& sc, double x, double y, double z);
// The last half metre before run A's first riser. The deployed hook stops
// NeuPAN there and drives the gait blind; without that, a point-set planner
// sees the next tread as a wall and parks 0.3 m short of it forever.
bool atStairMouth(const Scenario& sc, double x, double y, double z) {
  if (!sc.flights_only) return false;
  for (int k = 0; k < 5; ++k) {
    const double z0 = 3.40 * k;
    if (z < z0 - 0.05 || z > z0 + 0.18) continue;
    // 2.30, not 2.55: the four-storey run stalled 3 cm outside the window, on
    // the last centimetres before the first riser, where the tread reads as a
    // wall and nothing moves.
    if (x > 2.30 && x < 3.12 && y > 1.95 && y < 2.85) return true;
  }
  return false;
}

// relaxed widens run B's top edge from 3.30 to 3.45 m. Only the point-set
// planner needs it: a grid planner steps off the last tread onto the slab on
// its own, and blind-driving it there cost two runs that used to finish.
bool inFlight(const Scenario& sc, double x, double y, double z,
              bool relaxed = false) {
  if (!sc.flights_only) return inStair(sc, x, y, z);
  // A flight is only a flight while you are still on it. Without the height
  // test the body stayed "on run B" after stepping onto the second-floor slab,
  // and the run's centre line ends right under it - it chased its own mid-point
  // and spiralled for the rest of the run.
  // Storeys stack at 3.40 m and repeat the same double-run layout. Check every
  // storey rather than the one the height merely floors into: the body stands
  // on the slab at 3.42 m, which floors to storey 1, while still finishing the
  // run that belongs to storey 0. Picking one storey kept it "on the flight"
  // forever, chasing a centre line that ends under its feet.
  const double H = 3.40;
  for (int k = 0; k < 5; ++k) {
  const double z0 = H * k;
  if (z < z0 - 0.05 || z > z0 + (relaxed ? 3.45 : 3.35)) continue;
  if (x > 2.45 && x < 5.80 && y > 1.95 && y < 2.85 &&
      z > z0 + 0.10 && z < z0 + 1.95) return true;  // run A
  // Run B's hand-over starts at the landing mouth, not at the first tread.
  // DWA would not push the body the last 0.4 m from the landing into the
  // flight; the hook drives blind down the centre line and has no such
  // hesitation, so let it take over there.
  // The body stands on the next storey's slab at 3.42 while still finishing this
  // run. A point-set planner parks on the last tread if the hand-over has lapsed
  // by then, so NeuPAN gets the wider window; a grid planner does not need it.
  const double xb = relaxed ? 2.35 : 2.85;
  if (x > xb && x < 6.85 && y > 0.75 && y < 1.68 &&
      z > z0 + 1.60 && z < z0 + (relaxed ? 3.45 : 3.30)) return true;  // run B
  }
  return false;
}

bool inStair(const Scenario& sc, double x, double y, double z) {
  for (const auto& q : sc.stair)
    if (x >= q.x0 && x <= q.x1 && y >= q.y0 && y <= q.y1 && z >= q.z0 && z <= q.z1)
      return true;
  return false;
}

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

// D. two floors: the office stair core, double run with a 1.2 m landing.
Scenario stairTwoFloor() {
  Scenario s;
  s.name = "stair_two_floor";
  s.desc = "两层办公楼：双跑楼梯（净宽 1.0 m）+ 1.70 m 转弯平台，每跑 9 级 × 0.189 m，层高 3.40 m；"
           "一层起点、二层目标，楼梯段由 stair_gait 接管";
  s.minx = -0.2; s.maxx = 7.0;
  s.miny = -1.6; s.maxy = 3.6;
  s.zmin = -0.20; s.zmax = 3.90;
  s.sx = 1.0; s.sy = 0.0;
  s.gx = 1.0; s.gy = 1.2;
  s.sz = 0.0;  s.gz = 3.40;

  const double RISE = 0.189, TREAD = 0.30, LAND = 1.70;
  s.obs.push_back(box(-0.2, -1.6, 3.0, 3.2, -0.10, 0.0));      // first floor slab
  s.obs.push_back(box(-0.2, -0.2, 3.0, 3.2, 3.30, 3.40));      // second floor slab
  for (int i = 0; i < 9; ++i) {                                // run A, +x
    const double x0 = 3.0 + i * TREAD;
    s.obs.push_back(box(x0, 1.90, x0 + TREAD, 2.90, 0.0, RISE * (i + 1)));
  }
  // The half-landing spans the full width between the runs (y 1.2-3.0). With
  // it stopping at y 1.8 the dog could not step off flight A's top onto the
  // return flight: everything south of 1.8 at x>5.7 was a hole in the floor.
  s.obs.push_back(box(5.7, 0.80, 6.9, 2.90, 0.0, LAND));        // landing
  for (int i = 0; i < 9; ++i) {                                // run B, -x
    const double x1 = 5.7 - i * TREAD;
    s.obs.push_back(box(x1 - TREAD, 0.80, x1, 1.80, 0.0, LAND + RISE * (i + 1)));
  }
  // 0.22 m continuous upstands on both sides of both runs (8.5.1). They stop
  // short of the landing so the turn from run A to run B stays open - that is
  // how the real balustrade ends, not a gap somebody cut.
  // Upstand plus the balustrade above it: 0.22 m is the retrofit spec, and the
  // site already has a glass panel to shoulder height. Modelling only the
  // 0.22 m lip made its 8 cm top a standable ledge, and A* walked the
  // balustrade instead of the tread.
  const double RAIL = 0.90;
  for (int i = 0; i < 9; ++i) {
    const double x0 = 3.0 + i * TREAD;
    const double top = RISE * (i + 1);
    s.obs.push_back(box(x0, 2.90, x0 + TREAD, 2.98, top - 0.04, top + RAIL));
    if (x0 < 5.3)
      s.obs.push_back(box(x0, 1.78, x0 + TREAD, 1.86, top - 0.04, top + RAIL));
    const double x1 = 5.7 - i * TREAD;
    const double top2 = LAND + RISE * (i + 1);
    s.obs.push_back(box(x1 - TREAD, 0.72, x1, 0.80, top2 - 0.04, top2 + RAIL));
    if (x1 - TREAD < 5.3)
      s.obs.push_back(box(x1 - TREAD, 1.80, x1, 1.88, top2 - 0.04, top2 + RAIL));
  }
  s.obs.push_back(box(5.7, 2.90, 6.9, 2.98, LAND - 0.04, LAND + RAIL));
  s.obs.push_back(box(6.84, 1.9, 6.92, 2.9, LAND - 0.04, LAND + RAIL));
  // envelope walls
  s.obs.push_back(box(7.0, 0.5, 7.1, 3.3, 0.0, 4.2));
  s.obs.push_back(box(-0.3, 3.2, 7.0, 3.3, 0.0, 4.2));
  s.obs.push_back(box(-0.3, -1.7, 7.0, -1.6, 0.0, 4.2));
  s.obs.push_back(box(-0.3, -0.2, -0.2, 3.2, 0.0, 4.2));
  // second floor guard rail, so a plan cannot walk off the slab edge
  s.obs.push_back(box(-0.2, 3.1, 3.0, 3.18, 3.40, 4.4));
  s.obs.push_back(box(-0.2, -0.28, 3.0, -0.20, 3.40, 4.4));

  s.stair.push_back({2.7, 7.0, 0.5, 3.1, 0.0, 3.6});  // from the stair mouth up
  // Centre lines, the same mechanism the four-storey harness uses: without
  // them A* shaves the inside of the tread and the dog walks the balustrade.
  s.corridors = {
    {3.0, 5.7, 2.40, 0.20, 0.00, 1.80, 3.0},
    {3.0, 5.7, 1.30, 0.20, 1.60, 3.45, 3.0}
  };

  // stands the operator walked Odin1 through: every level and along both runs
  // Stands the operator walked: every level, plus stands *in front of* each
  // run, 1-2 m back on the floor below. Standing on a tread is useless - the
  // next tread sits 0.56 rad above the lowest line and the sensor stops at
  // +0.30. From 1-2 m back the whole run falls inside the field of view, which
  // is exactly why the staircase gets surveyed before the mission starts.
  s.survey_pts = {
    {0.6, -0.6, 0.0}, {2.4, -0.6, 0.0},
    {1.80, 2.40, 0.0}, {2.30, 2.40, 0.0},          // looking up run A
    {3.60, 2.40, 0.0},
    {6.30, 2.40, LAND},                             // looking back down run B
    {5.90, 1.40, LAND},
    {1.80, 1.30, LAND * 2}, {2.40, 2.20, LAND * 2}, // second floor, looking back
    {0.60, 0.60, LAND * 2}
  };
  // Mission legs: stair mouth -> landing -> upper floor. Each one is planned
  // against whatever the map knows at that moment, which is how the real stack
  // works and the only way a mostly-unsurveyed staircase gets walked.
  s.via = {
    {3.20, 2.40, 0.05},                              // stair mouth
    {6.30, 2.40, 1.80},                              // landing, turn
    {6.10, 1.30, 1.72},                              // run B mouth, on the landing
    {2.20, 1.30, 3.40},                              // well inside the slab, so
                                                     // A* has to climb run B
    {1.00, 1.30, 3.40}                               // second floor
  };
  s.seed_stair_prior = true;
  s.flights_only = true;
  s.ztol = 0.10;
  s.limit = 140.0;
  return s;
}

// E. four storeys: the same office stair core, stacked three times. This is the
// case the branch is actually built for - a four-storey office block - and it
// exercises the part a single flight cannot: a hand-over zone that has to know
// which storey it is on, and a mission that has to chain three of them.
Scenario fourFloor() {
  Scenario s;
  s.name = "four_floor";
  s.desc = "四层办公楼：同一楼梯芯垂直叠三层（双跑 × 9 级 × 0.189 m、净宽 1.0 m、层高 3.40 m），"
           "一层起点 → 四层目标，连爬三个层间，共 6 跑 54 级";
  // The landing is 1.60 m deep, not 1.20: a real stair hall always gives a full
  // tread width plus room to swing round, and this body is 0.52 m long - at
  // 1.20 m it had 0.55 m to the outer wall and turned by scraping along it.
  s.minx = -0.2; s.maxx = 7.4;
  s.miny = -1.6; s.maxy = 3.6;
  s.zmin = -0.20; s.zmax = 10.70;
  s.sx = 1.0; s.sy = 0.0; s.sz = 0.0;
  s.gx = 1.0; s.gy = 1.30; s.gz = 10.20;

  const double RISE = 0.189, TREAD = 0.30, LAND = 1.70, H = 3.40, RAIL = 0.90;

  for (int f = 0; f <= 3; ++f)                       // four floor slabs
    s.obs.push_back(box(-0.2, -0.2, 3.0, 3.2, H * f - 0.10, H * f));

  // Alternating runs, scissor-stair fashion: storey k uses one side of the core
  // for its up-run, storey k+1 uses the other. Stacking both storeys' flights
  // at the same (x, y) put the tread of one 3.4 m directly above the other, and
  // the body would not commit to the return flight - the map reads the two as
  // one column.
  for (int k = 0; k < 3; ++k) {                      // three storeys of flights
    const double z0 = H * k;
    // Same side on every storey. Alternating them scissor-fashion puts the
    // next storey's up-run directly over this storey's return flight, and the
    // body then climbs into its underside.
    const double ya0 = 1.90;                       // run A band
    const double ya1 = ya0 + 1.00;
    const double yb0 = 0.80;                       // run B band
    const double yb1 = yb0 + 1.00;
    for (int i = 0; i < 9; ++i) {                    // run A, +x
      const double x0 = 3.0 + i * TREAD;
      // One riser thick. Solid-from-the-floor is fine on a two-storey scene,
      // where nothing is below it, but stacked it fills the air above the
      // storey below's return flight: the body climbing x 4.5 at z 3.2 walks
      // straight into the column that reaches down to 3.40.
      s.obs.push_back(box(x0, ya0, x0 + TREAD, ya1,
                          z0 + RISE * i, z0 + RISE * (i + 1)));
    }
    s.obs.push_back(box(5.7, 0.80, 7.30, 2.90, z0, z0 + LAND));      // landing
    for (int i = 0; i < 9; ++i) {                    // run B, -x
      const double x1 = 5.7 - i * TREAD;
      s.obs.push_back(box(x1 - TREAD, yb0, x1, yb1,
                          z0 + LAND + RISE * i, z0 + LAND + RISE * (i + 1)));
    }
    const bool no_up = getenv("NAVSIM_NO_RAIL") != nullptr;
    for (int i = 0; i < 9; ++i) {                    // upstands, 8.5.1
      const double x0 = 3.0 + i * TREAD;
      const double top = z0 + RISE * (i + 1);
      if (!no_up) {
        s.obs.push_back(box(x0, ya1, x0 + TREAD, ya1 + 0.08, top - 0.04, top + RAIL));
        if (x0 < 5.3)
          s.obs.push_back(box(x0, ya0 - 0.08, x0 + TREAD, ya0, top - 0.04, top + RAIL));
      }
      const double x1 = 5.7 - i * TREAD;
      const double top2 = z0 + LAND + RISE * (i + 1);
      if (!no_up) {
        s.obs.push_back(box(x1 - TREAD, yb0 - 0.08, x1, yb0, top2 - 0.04, top2 + RAIL));
        if (x1 - TREAD < 5.3)
          s.obs.push_back(box(x1 - TREAD, yb1, x1, yb1 + 0.08, top2 - 0.04, top2 + RAIL));
      }
    }
    // One upstand per side of the landing, on the outer edge of each run.
    s.obs.push_back(box(5.7, ya1, 7.30, ya1 + 0.08, z0 + LAND - 0.04, z0 + LAND + RAIL));
    s.obs.push_back(box(5.7, yb0 - 0.08, 7.30, yb0, z0 + LAND - 0.04, z0 + LAND + RAIL));
    s.obs.push_back(box(7.24, 1.9, 7.32, 2.9, z0 + LAND - 0.04, z0 + LAND + RAIL));
  }

  // envelope walls, full height
  s.obs.push_back(box(7.4, 0.5, 7.5, 3.3, 0.0, 10.60));
  s.obs.push_back(box(-0.3, 3.2, 7.4, 3.3, 0.0, 10.60));
  s.obs.push_back(box(-0.3, -1.7, 7.4, -1.6, 0.0, 10.60));
  s.obs.push_back(box(-0.3, -0.2, -0.2, 3.2, 0.0, 10.60));
  for (int f = 1; f <= 3; ++f) {                     // slab guard rails
    const double z = H * f;
    s.obs.push_back(box(-0.2, 3.1, 3.0, 3.18, z, z + 1.0));
    s.obs.push_back(box(-0.2, -0.28, 3.0, -0.20, z, z + 1.0));
  }

  s.stair.push_back({2.7, 7.4, 0.5, 3.1, 0.0, 10.20});
  for (int k = 0; k < 3; ++k) {
    const double z0 = H * k;
    const double ca = 2.40;
    const double cb = 1.30;
    s.corridors.push_back({3.0, 5.7, ca, 0.20, z0, z0 + 1.85, 3.0});
    s.corridors.push_back({3.0, 5.7, cb, 0.20, z0 + 1.60, z0 + 3.50, 3.0});
  }

  // stands: in front of each run on every level, plus two floor stands
  for (int f = 0; f <= 3; ++f) {
    const double z = H * f;
    const double ca = 2.40;
    const double cb = 1.30;
    s.survey_pts.push_back({1.80, ca, z});
    s.survey_pts.push_back({3.60, ca, z});
    if (f < 3) s.survey_pts.push_back({6.70, ca, z + LAND});
    s.survey_pts.push_back({1.80, cb, z});
  }
  s.survey_pts.push_back({0.60, -0.60, 0.0});
  s.survey_pts.push_back({1.80, 1.30, H});

  // Mission legs, three storeys worth. Each run is its own leg so A* only ever
  // has to climb 1.7 m at a time, which is what makes 600k iterations enough.
  for (int k = 0; k < 3; ++k) {
    const double z0 = H * k;
    const double ca = 2.40;
    const double cb = 1.30;
    s.via.push_back({3.18, ca, z0 + 0.05});          // stair mouth
    s.via.push_back({5.95, ca, z0 + 1.80});          // just onto the landing
    s.via.push_back({6.65, ca, z0 + 1.77});          // turn point, 0.59 m off the rail
    // Make the lateral shift its own leg. With one leg from the landing to the
    // slab below, the arrival tolerance (0.45 m) is satisfied while the body is
    // still on the up-run's top tread at y 1.71, and the next leg then starts
    // from there - where the cheapest route to the goal is back DOWN the flight
    // it just climbed. Standing off the return flight's mouth first leaves only
    // one way in.
    s.via.push_back({6.65, cb, z0 + 1.74});          // shift across the landing
    s.via.push_back({5.30, cb, z0 + 1.78});          // return flight mouth
    s.via.push_back({2.20, cb, z0 + H});             // next slab
    // Turning room. Stepping off the last tread leaves the body facing west
    // with the return flight 0.65 m to its east: a differential drive needs
    // 0.71 m of arc to come round, so every candidate clipped a riser and the
    // planner chose to stand still for the rest of the run. Backing off to the
    // stair hall's west side first opens the turn.
    s.via.push_back({1.60, cb, z0 + H});             // back off, clear of the flight
    s.via.push_back({1.60, ca, z0 + H});             // face the next up-run
  }
  s.via.push_back({1.00, 1.30, H * 3});              // fourth floor

  s.seed_stair_prior = true;
  s.flights_only = true;
  s.ztol = 0.10;
  s.limit = 300.0;
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
             const VoxelGrid* prior = nullptr, const Scenario* sc = nullptr) {
  g.clear();
  // Live sweeps stack a solid slab inside the stair core for the same reason the
  // surveyed pass did: a ray that grazes each riser from outside walks up the
  // vertical face, and the hits pile into a wall where there are nine thin
  // treads. Whether this map is built once (survey) or every 0.5 s (mission),
  // only the lowest hit in a column is real - that is the tread under it. Left
  // unfixed, the two-storey run got away with it because it finishes before the
  // slab closes over the run; a four-storey stack stalls on its first return
  // flight with the planner reporting no path at all.
  if (sc && !sc->stair.empty()) {
    std::map<std::pair<long, long>, double> low;
    for (const auto& p : pts) {
      if (!inStair(*sc, p.x, p.y, p.z)) { stampPoint(g, p); continue; }
      const std::pair<long, long> key(
          (long)std::floor(p.x * 10.0 + 0.5), (long)std::floor(p.y * 10.0 + 0.5));
      auto it = low.find(key);
      if (it == low.end() || p.z < it->second) low[key] = p.z;
    }
    for (const auto& kv : low) {
      Point q;
      q.x = (double)kv.first.first / 10.0;
      q.y = (double)kv.first.second / 10.0;
      q.z = kv.second;
      stampPoint(g, q);
    }
  } else {
    for (const auto& p : pts) stampPoint(g, p);
  }
  if (prior) {
    for (int iz = 0; iz < g.nz(); ++iz)
      for (int iy = 0; iy < g.ny(); ++iy)
        for (int ix = 0; ix < g.nx(); ++ix) {
          // Copy the whole known state, not just the occupied cells: the free
          // run above a floor is what makes a cell standable, and the live
          // sweep only covers 4 m around the dog.
          const uint8_t v = prior->at(ix, iy, iz);
          if (v == VoxelGrid::kUnknown) continue;
          // The surveyed structure wins over a fresh free cell. One sweep from
          // the stair mouth stamps the tread's front edge and then marks 20
          // cells above it free - including the cell the tread itself sits in.
          // Letting that overwrite the prior erases the very step the dog has
          // to climb, and no later sweep puts it back.
          if (v == VoxelGrid::kOccupied) {
            // Keep the prior's *surfaces*; let the live sweep clear its bulk.
            // A storey's stair core is seeded solid all the way up, and the
            // body only ever sees the first 4 m of it. Wherever the sweep did
            // reach and reported air, the prior's interior has to give way -
            // otherwise a solid column 1.2 m over the trunk gets planned
            // through, which is exactly why a four storey stack never climbed
            // past its first landing.
            const int up = (iz + 1 < g.nz()) ? iz + 1 : iz;
            const bool is_surface = prior->at(ix, iy, up) != VoxelGrid::kOccupied;
            if (!is_surface && g.at(ix, iy, iz) == VoxelGrid::kFree) continue;
            g.set(ix, iy, iz, v);
            continue;
          }
          if (g.at(ix, iy, iz) != VoxelGrid::kUnknown) continue;
          g.set(ix, iy, iz, v);
        }
  }
  // 0.20 m of inflation, plus the 0.20 m envelope sampled by blocked2D, is
  // the conservative pair for a trunk half width of 0.17 m: 3 cells on both
  // sides closed a 1.6 m corridor completely once a bin was in it, which the
  // real dog walks through.
  if (!getenv("NAVSIM_NO_INFLATE"))
    g.inflate(2, 0, /*skip_standing_surfaces=*/true);

  // Inside a stair core the horizontal inflation has to go. A 0.30 m tread
  // with a 0.20 m side expansion from the next riser leaves two thirds of the
  // standing cell inflated, and the dog cannot climb a staircase the planner
  // refuses to enter. Stair runs are executed by stair_gait anyway; the
  // upstands and the walls are the real constraint there.
  if (sc && !sc->stair.empty()) {
    for (int iz = 0; iz < g.nz(); ++iz)
      for (int iy = 0; iy < g.ny(); ++iy)
        for (int ix = 0; ix < g.nx(); ++ix) {
          if (g.at(ix, iy, iz) != VoxelGrid::kInflated) continue;
          double wx, wy, wz;
          g.indexToWorld(ix, iy, iz, wx, wy, wz);
          if (inStair(*sc, wx, wy, wz)) g.set(ix, iy, iz, VoxelGrid::kFree);
        }
  }
}

// Survey stands: one sweep every 2.4 m, which is roughly what an operator gets
// by walking Odin1 around the floor once before handing the dog a mission.
void surveyMap(const Scenario& sc, const SensorModel& sm, std::mt19937& rng,
               VoxelGrid& prior) {
  prior.configureFromBounds(0.10, sc.minx - 0.6, sc.miny - 0.6, sc.zmin,
                            sc.maxx + 0.6, sc.maxy + 0.6, sc.zmax, 0.30);
  prior.clear();
  std::vector<Point> one;
  // Multi-floor needs stands on every level, including the stair treads: a
  // single ground-level walk-around leaves the upper slab and the middle of
  // the staircase unknown, and the very first plan then has nowhere to land.
  std::vector<std::array<double, 3>> stands;
  if (!sc.survey_pts.empty()) {
    stands = sc.survey_pts;
  } else {
    for (double bx = sc.minx + 1.2; bx <= sc.maxx - 0.6; bx += 2.4)
      for (double by = sc.miny + 1.2; by <= sc.maxy - 0.6; by += 2.4)
        stands.push_back({bx, by, 0.0});
  }
  std::map<std::pair<int, int>, double> stair_low;
  if (sc.seed_stair_prior) {
    for (const auto& o : sc.obs) {
      if (o.kind != 0) continue;
      const double bw = o.a[2] - o.a[0], bd = o.a[3] - o.a[1];
      if (std::min(bw, bd) < 0.25) continue;        // skips the 0.08 m upstands
      const double cx = 0.5 * (o.a[0] + o.a[2]), cy = 0.5 * (o.a[1] + o.a[3]);
      if (!inStair(sc, cx, cy, std::min(o.z1, 3.55))) continue;
      for (double px = o.a[0]; px <= o.a[2] + 1e-6; px += 0.10)
        for (double py = o.a[1]; py <= o.a[3] + 1e-6; py += 0.10)
          prior.insertSurfacePoint(px, py, o.z1 - 0.01, 20);
    }
  }
  for (const auto& st : stands) {
      scan(sc.obs, 0.0, st[0], st[1], st[2], 0.0, sm, rng, one, sc.ztol);
      // Same stamping as the live sweep: the 3x3 fill of the ground band is
      // what turns 240 near-field returns into a walkable floor.
      // In the stair core keep only the LOWEST hit per column. Seen from 3.6 m
      // away the sweep grazes the risers and its hits run up the vertical face
      // as a continuous column - measured at x 5.40 / y 1.30 as 16 filled cells
      // from z 1.60 to 2.35 where the true treads are three thin layers topping
      // out at 2.08. The lowest hit is the tread the column stands on, which is
      // the only surface that should be there.
      for (const auto& p : one) {
      int six = 0, siy = 0, siz = 0;
      if (sc.seed_stair_prior && inStair(sc, p.x, p.y, p.z) &&
          prior.worldToIndex(p.x, p.y, p.z, six, siy, siz)) {
        auto key = std::make_pair(six, siy);
        auto it = stair_low.find(key);
        if (it == stair_low.end() || p.z < it->second) stair_low[key] = p.z;
        continue;
      }
      prior.insertSurfacePoint(p.x, p.y, p.z, 20);
        if (p.z > -0.09 && p.z < 0.06)
          for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
              prior.insertSurfacePoint(p.x + dx * prior.resolution(),
                                       p.y + dy * prior.resolution(), p.z, 20);
      }
  }
  for (const auto& kv : stair_low) {
    double wx = 0.0, wy = 0.0, wz = 0.0;
    prior.indexToWorld(kv.first.first, kv.first.second, 0, wx, wy, wz);
    prior.insertSurfacePoint(wx, wy, kv.second, 20);
  }
}

// Plan cell for a world pose: the free cell sitting on the floor.
bool planCell(const VoxelGrid& g, double x, double y, double base_z, VoxelKey& out) {
  int ix, iy, iz;
  if (!g.worldToIndex(x, y, base_z + kBaseZ, ix, iy, iz)) return false;
  // groundBelow already steps one cell up from the surface it found, i.e.
  // it returns the cell the body occupies - not the floor cell.
  const int stand = g.groundBelow(ix, iy, iz);
  if (stand >= 0) {
    out.x = ix; out.y = iy; out.z = stand;
    return true;
  }
  // A survey leaves a hole right under the operator's feet: the sensor sits
  // 0.45 m up and cannot see the floor closer than ~0.8 m, so the cell the
  // dog is told to drive to can be surface-less while its neighbours are
  // fine. Snap to the nearest column that does have one - the same courtesy
  // A* already extends through snap_search_radius_cells.
  for (int r = 1; r <= 3; ++r)
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) != r && std::abs(dy) != r) continue;
        const int s2 = g.groundBelow(ix + dx, iy + dy, iz);
        if (s2 < 0) continue;
        out.x = ix + dx; out.y = iy + dy; out.z = s2;
        return true;
      }
  return false;
}

// Projected 2D occupancy for the local rollout. Only the band the body
// actually occupies counts, so a tabletop at 0.75 m does not block the robot
// walking underneath it - which is exactly the property the 3D grid buys us.
// Single grid cell in the trunk band 0.12 .. 0.42 m.
bool cellBlocked(const VoxelGrid& g, double wx, double wy, double base_z) {
  int ix, iy, iz;
  // The band is the trunk, not the legs: URDF puts the collision box centre
  // 0.32 m above the standing face with a 0.114 m height, so 0.24 .. 0.44 m
  // covers it. The old 0.12 .. 0.42 band counted the legs and therefore read
  // every 0.189 m stair riser as an obstacle the dog could not climb.
  if (!g.worldToIndex(wx, wy, base_z + 0.24, ix, iy, iz)) return true;  // outside = blocked
  for (int dz = 0; dz <= 2; ++dz) {
    const int jz = iz + dz;  // base + 0.24 .. 0.44 m: the trunk
    if (jz >= g.nz()) break;
    const uint8_t v = g.at(ix, iy, jz);
    if (v == VoxelGrid::kOccupied || v == VoxelGrid::kInflated) return true;
  }
  return false;
}

// Body envelope, not a point. Checking only the centre let the trunk corner
// clip a 1.2 m door frame while the centre line still read as free, which is
// the one failure a real Go2 cannot recover from at walking speed.
// Distance from a point to the nearest live cloud return in the body's height
// band. NeuPAN constrains the robot against the point set itself (dune_max_num
// points, collision_threshold 0.05), so this replaces the inflated grid for
// that planner. Points above the trunk are ignored - they cannot be hit.
double nearestPointDist(const std::vector<Point>& cl, double px, double py,
                        double base_z) {
  double best = 1e9;
  // dune_max_num is 100 in planner.yaml, but that is 100 points after the
  // obstacle filter, not a blind 1-in-N of the sweep. Striding over the raw
  // cloud dropped most of the pedestrian's returns - a 0.44 m cylinder two
  // metres away is only a handful of rays - and the dog walked straight into
  // him. Keep every other in-band point instead of every fifth raw point.
  const int stride = 2;
  for (size_t i = 0; i < cl.size(); i += stride) {
    const auto& q = cl[i];
    // The floor is a return like any other, and standing on it puts it at
    // distance zero - which made every candidate illegal and parked the dog
    // for the whole run. The deployed stack feeds NeuPAN the obstacle layer
    // (pointcloud_to_laserscan over sliding_map), not the ground, so anything
    // within 0.12 m of the standing face is treated as ground here too.
    if (q.z < base_z + 0.12 || q.z > base_z + 0.55) continue;
    const double dx = q.x - px, dy = q.y - py;
    const double d = std::hypot(dx, dy);
    if (d < best) best = d;
  }
  return best;
}

bool blocked2D(const VoxelGrid& g, double wx, double wy, double base_z) {
  // The grid is already inflated by two cells (0.20 m) for the 0.17 m half
  // width, so the centre point IS the footprint test. A second 0.20 m envelope
  // charged that radius twice: a 0.70 m doorway then had
  // 0.70 - 0.40 - 0.40 = -0.10 m of usable width and not one cell inside it
  // was legal, so the dog could not pass whatever the cost function said.
  return cellBlocked(g, wx, wy, base_z);
}

// Ground-truth footprint test: OBB (robot) against every live primitive.
bool hitsTruth(const std::vector<Obstacle>& obs, double t, double x, double y,
               double base_z, double yaw) {
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
    // Trunk top sits at kBaseZ + 0.057 = 0.377 above the standing surface;
    // 0.44 reached 6 cm into the underside of the storey above, so on a four
    // storey stack every return flight grazed the one overhead.
    if (o.z0 > base_z + 0.38) continue;   // entirely above the trunk
    // Trunk underside sits at kBaseZ - 0.057 = 0.263 above the standing
    // surface; 0.24 let the belly clip the riser one step ahead, which on a
    // 54-riser climb is a collision every other step.
    if (o.z1 < base_z + 0.32) continue;   // entirely below it (a lower riser)
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
  double x = 0, y = 0, z = 0, yaw = 0;
  double vx = 0, vy = 0, wz = 0;
  bool hit = false;
};

struct WP { double x, y, z; };  // route point incl. standing height

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
  std::vector<WP> global_path;
  double final_x = 0, final_y = 0;
  // The planner's own view of the stair core, sampled once near t=30 s on the
  // return-run centre line. This is the map that decides whether a flight is
  // walkable, and it is the thing to look at when two storeys climb and four
  // do not.
  double probe_t = -1, probe_y = 1.30;
  std::vector<std::vector<int>> probe_cells;   // rows: z descending, cols: x
  // The local cost window: what the local planner is allowed to look at, and
  // only inside it are obstacles inflated. Samples are sparse in time (one per
  // mapping refresh) and store cells as offsets on a 0.10 m lattice, so the
  // page can rebuild the same square while the run plays back.
  struct LocalWin {
    double t = 0, cx = 0, cy = 0, r = 2.0;
    std::vector<std::array<int, 2>> cells;     // dx, dy from the window corner
  };
  std::vector<LocalWin> wins;
};

// Coarse clearance: distance from the body box to the nearest live primitive.
double clearance(const std::vector<Obstacle>& obs, double t, double x, double y,
                 double base_z) {
  double best = 6.0;
  for (const auto& o : obs) {
    if (!o.present(t)) continue;
    if (o.z0 > base_z + 0.44) continue;   // above the trunk
    if (o.z1 < base_z + 0.24) continue;   // below it
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
  grid.configureFromBounds(0.10, sc.minx - 0.6, sc.miny - 0.6, sc.zmin,
                           sc.maxx + 0.6, sc.maxy + 0.6, sc.zmax, 0.30);

  VoxelGrid prior;
  surveyMap(sc, sm, rng, prior);

  std::vector<Obstacle> obs = sc.obs;
  std::vector<Point> cloud;           // current sweep
  std::vector<Point> accum;           // last few sweeps, what the map is built from

  double x = sc.sx, y = sc.sy, yaw = std::atan2(sc.gy - y, sc.gx - x);
  double base_z = sc.sz;              // standing face under the trunk
  double vx = 0, vy = 0, wz = 0;
  const double dt_sim = 0.02;
  const double ctrl = 0.10;
  double ctrl_accum = 0;

  // initial sweep so the first plan has something to work with
  scan(obs, 0.0, x, y, base_z, yaw, sm, rng, cloud, sc.ztol);
  accum = cloud;
  rebuild(grid, accum, &prior, &sc);
  out.scans = 1;
  out.clouds.push_back(cloud);
  out.cloud_t.push_back(0.0);

  AStar3D planner;
  AStar3D::Params params;
  params.max_iterations = 600000;  // two-storey grids are ~4x the cells
  params.require_ground_support = true;
  // Optimistic planning inside a stair core only: the flat scenes keep the
  // strict FREE rule, which is what keeps mincl honest there.
  params.allow_unknown_cells = sc.seed_stair_prior;
  params.snap_search_radius_cells = 12;
  params.max_step_z_cells = 3;
  params.corridors = sc.corridors;

  // The mission is handed over one leg at a time, exactly the way
  // floor_manager hands a floor transition to the planner. A single plan all
  // the way to the upper floor deadlocks instead: the staircase is mostly
  // unknown until the dog walks it, and a failed plan leaves the dog standing
  // still, so it never sweeps the part it could not plan.
  std::vector<std::array<double, 3>> legs;
  if (!sc.via.empty()) {
    legs = sc.via;
  } else {
    legs.push_back({sc.gx, sc.gy, sc.gz});
  }
  size_t leg = 0;
  auto leg_goal = [&]() -> const std::array<double, 3>& {
    return legs[std::min(leg, legs.size() - 1)];
  };

  std::vector<WP> gpath;                          // current route
  std::vector<WP> first_path;                     // route as first planned
  double last_plan_t = -10.0;
  bool have_plan = false;

  auto replan = [&](double t) {
    VoxelKey s, g;
    if (!planCell(grid, x, y, base_z, s)) {
      out.events.push_back({t, "规划失败：起点不在网格内"});
      return false;
    }
    if (!planCell(grid, leg_goal()[0], leg_goal()[1], leg_goal()[2], g)) {
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
        first_path.push_back({wx, wy, wz});
      }
    gpath.clear();
    for (const auto& k : r.path) {
      double wx, wy, wz;
      grid.indexToWorld(k.x, k.y, k.z, wx, wy, wz);
      gpath.push_back({wx, wy, wz});
    }
    have_plan = true;
    last_plan_t = t;
    if (std::getenv("NAVSIM_TRACE"))
      std::fprintf(stderr,
                   "  [plan] t=%.1f expanded=%d pts=%d from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f)\n",
                   t, r.expanded, (int)gpath.size(), x, y, base_z,
                   leg_goal()[0], leg_goal()[1], leg_goal()[2]);
    if (std::getenv("NAVSIM_TRACE")) {
      for (const auto& col : {std::array<double,2>{2.85,2.35}, std::array<double,2>{3.15,2.40},
                              std::array<double,2>{3.35,2.40}, std::array<double,2>{3.15,2.20}}) {
        std::fprintf(stderr, "  staircol (%.2f,%.2f):", col[0], col[1]);
        for (double wz = 0.05; wz < 0.70; wz += 0.10) {
          int a=-1,b=-1,c=-1;
          if (grid.worldToIndex(col[0], col[1], wz, a, b, c))
            std::fprintf(stderr, " %d@%.2f", (int)grid.at(a,b,c), wz);
        }
        std::fprintf(stderr, "\n");
      }
      std::fprintf(stderr, "  leg %d plan t=%.1f from (%.2f,%.2f,%.2f) n=%d:", (int)leg, t, x, y, base_z, (int)gpath.size());
      for (int i = 0; i < (int)gpath.size() && i < 12; ++i)
        std::fprintf(stderr, " (%.2f,%.2f,%.2f)", gpath[i].x, gpath[i].y, gpath[i].z);
      std::fprintf(stderr, "\n");
    }
    return true;
  };

  {
    VoxelKey sk, gk;
    const bool sok = planCell(grid, x, y, base_z, sk);
    const bool gok = planCell(grid, sc.gx, sc.gy, sc.gz, gk);
    std::fprintf(stderr, "[%s] grid n=(%d,%d,%d) origin_z=%.2f  start ok=%d (%d,%d,%d)"
                 "  goal ok=%d (%d,%d,%d)\n", sc.name.c_str(), grid.nx(), grid.ny(),
                 grid.nz(), grid.originZ(), (int)sok, sk.x, sk.y, sk.z,
                 (int)gok, gk.x, gk.y, gk.z);
    if (gok) {
      std::fprintf(stderr, "  goal col z-4..z+4:");
      for (int z = std::max(0, gk.z - 4); z < std::min(grid.nz(), gk.z + 5); ++z)
        std::fprintf(stderr, " %d", grid.at(gk.x, gk.y, z));
      std::fprintf(stderr, "\n");
    }
    for (const auto& col : {std::array<double,2>{2.95,1.05}, std::array<double,2>{3.15,2.40},
                            std::array<double,2>{4.05,2.40}, std::array<double,2>{4.05,2.20}}) {
      std::fprintf(stderr, "  col (%.2f,%.2f) z=0.05..1.60:", col[0], col[1]);
      for (double wz = 0.05; wz < 1.62; wz += 0.10) {
        int a=-1,b=-1,c=-1;
        if (grid.worldToIndex(col[0], col[1], wz, a, b, c))
          std::fprintf(stderr, " %d@%.2f", (int)grid.at(a,b,c), wz);
      }
      std::fprintf(stderr, "\n");
    }
    for (int i = 0; i < 9; ++i) {
      const double sx3 = 3.15 + i * 0.30, sy3 = 2.4;
      VoxelKey k;
      const bool ok3 = planCell(grid, sx3, sy3, 0.189 * (i + 1) - 0.10, k);
      std::fprintf(stderr, "  step %d at z=%.2f ok=%d cell=%d\n", i, 0.189 * (i + 1), (int)ok3,
                   ok3 ? (int)grid.at(k.x, k.y, k.z) : -9);
    }
  }
  replan(0.0);
  out.replans = 1;

  // nearest point on the global route
  auto routeError = [&](double px, double py) {
    double best = 1e9;
    for (const auto& q : gpath) best = std::min(best, std::hypot(px - q.x, py - q.y));
    return best;
  };

  // DWA over the perceived grid.
  struct Cand { double vx, vy, wz; };
  std::vector<Cand> cands;
  if (sc.planner == "neupan") {
    // Differential drive: no lateral velocity at all. Speeds are the Go2
    // numbers from planner.yaml (0.625 m/s, 0.875 rad/s).
    for (double a : {-0.625, -0.40, -0.20, 0.0, 0.20, 0.40, 0.625})
      for (double c : {-0.875, -0.45, 0.0, 0.45, 0.875})
        cands.push_back({a, 0.0, c});
  } else {
    for (double a : {-0.15, 0.0, 0.15, 0.35, 0.55})
      for (double b : {-0.15, 0.0, 0.15})
        for (double c : {-1.1, -0.55, 0.0, 0.55, 1.1})
          cands.push_back({a, b, c});
  }

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

    // The local planner consumes the live sweep, and the deployed stack feeds
    // NeuPAN at 10 Hz. Refreshing it at 2 Hz let a 0.28 m/s pedestrian close a
    // whole body width between two samples: the dog saw him at 0.6 m, committed,
    // and by the next sweep he was at 0.2 m. Mapping keeps its own 0.5 s clock.
    if (step % 5 == 0) {
      scan(obs, t, x, y, base_z, yaw, sm, rng, cloud, sc.ztol);
    }

    // 0.5 s mapping refresh
    if (step % 25 == 0) {  // 0.5 s: mapping refresh, and one local window
      {
        // Only the box travels: the global grid is inflated once, but what the
        // local planner may use is the part of it inside 2 m of the robot. This
        // records exactly that slice, including which cells are solid (a tread
        // is solid, the space above it is not).
        Run::LocalWin lw;
        lw.t = t; lw.cx = x; lw.cy = y; lw.r = 2.0;
        const double step_m = 0.10;
        const int side = static_cast<int>(std::floor(2.0 * lw.r / step_m)) + 1;
        lw.cells.reserve(64);
        for (int a = 0; a < side; ++a) {
          for (int b = 0; b < side; ++b) {
            const double wx = lw.cx - lw.r + a * step_m;
            const double wy = lw.cy - lw.r + b * step_m;
            if (std::hypot(wx - lw.cx, wy - lw.cy) > lw.r) continue;
            int ix = 0, iy = 0, iz = 0;
            if (!grid.worldToIndex(wx, wy, base_z + 0.24, ix, iy, iz)) continue;
            bool solid = false;
            for (int dz = 0; dz <= 2; ++dz) {
              const int jz = iz + dz;
              if (jz >= grid.nz()) break;
              const uint8_t v = grid.at(ix, iy, jz);
              if (v == VoxelGrid::kOccupied || v == VoxelGrid::kInflated) { solid = true; break; }
            }
            if (solid) lw.cells.push_back({a, b});
          }
        }
        out.wins.push_back(lw);
      }
      scan(obs, t, x, y, base_z, yaw, sm, rng, cloud, sc.ztol);
      accum.insert(accum.end(), cloud.begin(), cloud.end());
      if (accum.size() > 16000) accum.erase(accum.begin(), accum.begin() + (accum.size() - 16000));
      rebuild(grid, accum, getenv("NAVSIM_NO_PRIOR") ? nullptr : &prior, &sc);
      // Sample the planner's grid just after the point where a stuck run gives
      // up, so the report can show the cells it was refusing to cross.
      if (!sc.stair.empty() && out.probe_t < 0 && t > 29.4) {
        out.probe_t = t;
        for (double z = 4.30; z >= -0.11; z -= 0.10) {
          std::vector<int> row;
          for (double xx = 2.70; xx <= 7.01; xx += 0.10) {
            int ix = 0, iy = 0, iz = 0;
            if (!grid.worldToIndex(xx, out.probe_y, z, ix, iy, iz)) row.push_back(-1);
            else row.push_back(static_cast<int>(grid.at(ix, iy, iz)));
          }
          out.probe_cells.push_back(row);
        }
      }
      if (getenv("NAVSIM_DUMP") && t > 29.4 && t < 30.6) {
        char ch[4] = {'.', ' ', '#', '+'};
        for (double yq : {1.30, 2.40}) {
          fprintf(stderr, "\n[dump] X-Z section at y=%.2f (t=%.1f), '.'=unknown ' '=free '#'=occupied '+'=inflated\n", yq, t);
          for (double z = 4.30; z >= -0.10; z -= 0.10) {
            fprintf(stderr, "  z%5.2f |", z);
            for (double xx = 2.70; xx <= 7.00; xx += 0.10) {
              int ix = 0, iy = 0, iz = 0;
              if (!grid.worldToIndex(xx, yq, z, ix, iy, iz)) { fprintf(stderr, "?"); continue; }
              const int v = static_cast<int>(grid.at(ix, iy, iz));
              fprintf(stderr, "%c", (v >= 0 && v <= 3) ? ch[v] : '?');
            }
            fprintf(stderr, "|\n");
          }
          fprintf(stderr, "          +");
          for (int c = 0; c < 32; ++c) fprintf(stderr, "-");
          fprintf(stderr, "\n           x2.70 -> 7.00 step 0.10\n");
        }
        fprintf(stderr, "\n[dump] column tops, y=1.30   x   grid   truth\n");
        for (double xx = 2.70; xx <= 7.01; xx += 0.30) {
          double gtop = -99.0, ttop = -99.0;
          const double stepz = grid.resolution();
          for (double z = -0.20; z <= 5.00; z += stepz) {
            int ix = 0, iy = 0, iz = 0;
            if (!grid.worldToIndex(xx, 1.30, z, ix, iy, iz)) continue;
            if (static_cast<int>(grid.at(ix, iy, iz)) == 2 && z > gtop) gtop = z;
          }
          for (const auto& o : sc.obs) {
            if (xx < o.a[0] || xx > o.a[2] || 1.30 < o.a[1] || 1.30 > o.a[3]) continue;
            if (o.z1 <= 5.00 && o.z1 > ttop) ttop = o.z1;
          }
          fprintf(stderr, "  x%5.2f  grid %6.2f  truth %6.2f   %s\n", xx, gtop, ttop,
                  (gtop > -50 && std::fabs(gtop - ttop) < 0.18) ? "ok" : "MISMATCH");
        }
        for (double xx : {5.40, 5.70, 6.30}) {
          fprintf(stderr, "  vert x%.2f y1.30:", xx);
          for (double z = 1.40; z <= 3.60; z += 0.05) {
            int ix = 0, iy = 0, iz = 0;
            if (!grid.worldToIndex(xx, 1.30, z, ix, iy, iz)) { fprintf(stderr, "?"); continue; }
            fprintf(stderr, "%d", static_cast<int>(grid.at(ix, iy, iz)));
          }
          fprintf(stderr, "   (z1.40 -> 3.60 step 0.05)\n");
        }
      }
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

      // Not gated on have_plan any more: the height now comes from the probes
      // instead of the route, so a leg can finish even while A* is unable to
      // plan across the flight. Before this the dog stood 0.26 m from its own
      // waypoint for thirty seconds because the arrival test never ran.
      {
        const auto& lg = leg_goal();
        const double dgoal = std::hypot(lg[0] - x, lg[1] - y);
        const double dheight = std::fabs(base_z - lg[2]);
        // Height matters: on a two-storey job two legs can share x,y and differ
        // only in z, so a 2D arrival test would fire on the wrong floor.
        if (dgoal < 0.45 && dheight < 0.35) {
          if (leg + 1 < legs.size()) {
            ++leg;
            have_plan = false;                 // force a plan for the new leg
            last_plan_t = -10.0;
            char buf[160];
            std::snprintf(buf, sizeof(buf), "第 %d 段到达（x=%.2f y=%.2f z=%.2f），转入第 %d 段",
                          (int)leg, x, y, base_z, (int)leg + 1);
            out.events.push_back({t, buf});
          } else {
            done = true;
            out.reached = true;
            out.events.push_back({t, "到达目标"});
          }
          vx = 0.0; vy = 0.0; wz = 0.0;

        } else if (inFlight(sc, x, y, base_z,
                             sc.planner == "neupan" &&
                                 sc.name.find("four") != std::string::npos) ||
                   (sc.planner == "neupan" && atStairMouth(sc, x, y, base_z))) {
          // Stair runs are executed by stair_gait, not by the local planner:
          // the hook stops /neupan, sends a constant forward /cmd_vel and lets
          // the gait mode do the climbing. Reproduce that here, otherwise DWA
          // would refuse to step onto a riser it can see as an obstacle.
          double bestd = 1e9;
          int bi = 0;
          for (int i = 0; i < (int)gpath.size(); ++i) {
            const double d = std::hypot(x - gpath[i].x, y - gpath[i].y);
            if (d < bestd) { bestd = d; bi = i; }
          }
          // Follow the run's centre line, not the route points. The route is
          // the shortest path and hugs the balustrade; stair_gait drives blind
          // down the middle of the flight, which is what the corridor weight in
          // the planner is there to encourage in the first place.
          const bool upper = (y > 1.80);
          // Every NeuPAN run needs the wider blind line, not just the four
          // storeys: the two-storey scene ends on that slab and parks on the
          // same boundary point.
          const bool relaxed = sc.planner == "neupan";
          // x=2.95 sits exactly on the hand-over boundary (x > 2.85), so the
          // goal was the point the body already stood on: atan2(0, 0) then spun
          // it in place for the rest of the run. Aim past the edge instead.
          const double cx_run = upper ? 5.98 : (relaxed ? 2.20 : 2.95);
          const double cy_run = upper ? 2.40 : 1.20;
          const double ang = std::atan2(cy_run - y, cx_run - x);
          double err = ang - yaw;
          while (err > M_PI) err -= 2 * M_PI;
          while (err < -M_PI) err += 2 * M_PI;
          // stair_gait drives forward blind. Turning while driving is what
          // makes it cut the corner, so stop and rotate first, like the real
          // hook's stop-motion step.
          vx = (std::fabs(err) > 0.55) ? 0.0 : 0.25;  // STAIR_SPEED
          vy = 0.0;                                   // linear.y hard-wired to 0
          wz = std::max(-1.0, std::min(1.0, 2.0 * err));
        } else {
          // which way is the route pointing? use the nearest route point a
          // little ahead of us, not the raw goal, so a detour is respected
          double bestd = 1e9, tx = leg_goal()[0], ty = leg_goal()[1];
          bool break_dwa = false;
          int bi = 0;
          for (int i = 0; i < (int)gpath.size(); ++i) {
            const double d = std::hypot(x - gpath[i].x, y - gpath[i].y);
            if (d < bestd) { bestd = d; bi = i; }
          }
          // An empty route makes size()-1 negative and reads garbage as the
          // aim point; the dog then turns for ever without moving. Fall back
          // to the leg goal, which is exactly where the session wants to go.
          if (!gpath.empty()) {
            const int ahead = std::min((int)gpath.size() - 1, bi + 8);
            tx = gpath[ahead].x;
            ty = gpath[ahead].y;
          }

          // Score progress towards the route, and score how much of the
          // heading error the rollout removed. A plain distance cost makes
          // "stand still" the cheapest option whenever the dog is pointed away
          // from the route: forward candidates gain nothing because they run
          // into the wall, and turn-only candidates gain nothing at all.
          const double d0 = std::hypot(tx - x, ty - y);
          double herr0 = std::atan2(ty - y, tx - x) - yaw;
          while (herr0 > M_PI) herr0 -= 2 * M_PI;
          while (herr0 < -M_PI) herr0 += 2 * M_PI;
          double best_cost = 1e18;
          Cand best{0, 0, 0};
          bool any = false;
          if (!break_dwa) {
          // Who is in the way, judged where he will be in 1.2 s. The rollouts
          // below only look 1.2 s ahead, but a pedestrian closing head-on
          // collides after about 3 s, so his future position never enters a
          // candidate's clearance - which is how dropping the old stop rule
          // made the dog walk straight into him. Instead of stopping him
          // afterwards, take "keep going" out of the candidate set and let the
          // cost function choose among stopping, shuffling sideways and
          // backing off.
          bool ped_ahead = false;
          if (sc.planner != "neupan") {   // NeuPAN has no forecast: he is a point
            for (const auto& o : obs) {          // the advanced copies, not spawn poses
              if (o.kind != 1 || !o.present(t)) continue;
              if (o.vx == 0.0 && o.vy == 0.0) continue;   // pillars are to walk round
              if (o.z0 > base_z + 0.44 || o.z1 < base_z + 0.24) continue;
              const double ox = o.a[0] + o.vx * 1.2;
              const double oy = o.a[1] + o.vy * 1.2;
              const double dx = ox - x, dy = oy - y;
              const double along = dx * std::cos(yaw) + dy * std::sin(yaw);
              const double side = -dx * std::sin(yaw) + dy * std::cos(yaw);
              if (along > -0.20 && along < 1.70 && std::fabs(side) < 0.60) {
                ped_ahead = true;
                break;
              }
            }
          }
          // Two passes: 1.2 s first, then 0.6 s when nothing survives. Tucked
          // against a door post every 1.2 s arc ends in the wall, so the long
          // pass parks the dog exactly where the pedestrian walks past it; the
          // short pass still sees the escape move.
          for (int pass = 0; pass < 2 && !any; ++pass) {
          // NeuPAN recedes over 10 x 0.1 s, i.e. a full 1.0 s, in one pass.
          const int nsteps = (sc.planner == "neupan") ? 10 : ((pass == 0) ? 6 : 3);
          best_cost = 1e18;
          for (const auto& c : cands) {
            // Going forward is off the table while a pedestrian is closing:
            // the 1.2 s horizon cannot see the meeting point, so a candidate
            // that looks clear now is the one that hits him.
            if (ped_ahead && c.vx > 0.0) continue;
            // roll out 1.2 s at 0.2 s steps (0.6 s on the fallback pass)
            double sx2 = x, sy2 = y, syaw2 = yaw;
            bool bad = false;
            double mincl = 9.9;
            for (int k = 0; k < nsteps; ++k) {
              const double h = 0.20;
              sx2 += (c.vx * std::cos(syaw2) - c.vy * std::sin(syaw2)) * h;
              sy2 += (c.vx * std::sin(syaw2) + c.vy * std::cos(syaw2)) * h;
              syaw2 += c.wz * h;
              if (sc.planner == "neupan") {
                // Hard constraint, not a cost term: a candidate whose body
                // would come closer than half width + 0.05 m to any live
                // return is simply illegal, the way collision_threshold works
                // in the PAN optimisation. Sampled every other step - the
                // point set is much denser than a grid and this is the hot
                // loop.
                const double d = nearestPointDist(cloud, sx2, sy2, base_z);
                mincl = std::min(mincl, std::max(0.0, d - kHalfWid));
                if (d < kHalfWid + 0.05) { bad = true; break; }
              } else {
                if (blocked2D(grid, sx2, sy2, base_z)) { bad = true; break; }
                // Score the clearance where the obstacle will be, not where it
                // is. A pedestrian walking at 0.28 m/s covers 0.34 m during a
                // 1.2 s rollout, which is a whole body diameter: judging him by
                // his present position is how the dog ends up stopping beside
                // him instead of short of him.
                mincl = std::min(mincl, clearance(obs, t + k * h, sx2, sy2, base_z));
              }
            }
            if (bad) continue;
            any = true;
            const double d2 = std::hypot(tx - sx2, ty - sy2);
            double herr2 = std::atan2(ty - sy2, tx - sx2) - syaw2;
            while (herr2 > M_PI) herr2 -= 2 * M_PI;
            while (herr2 < -M_PI) herr2 += 2 * M_PI;
            // Standing still is not free. Without this the clearance term
            // outweighs the small progress a 0.15 m/s step makes whenever the
            // dog is close to anything, and it parks for ever one body length
            // short of the next flight.
            // Turning in place is a stall too. Exempting it made "spiral on
            // the spot" cheaper than the one 0.15 m/s step that would finish
            // the leg, once the clearance term is large.
            // Turning in place is a stall only when the target is roughly
            // ahead. With the target behind, turning is the whole job, and
            // charging it as a stall leaves the body parked nose-to-wall
            // because forward is blocked and the turn is taxed.
            const bool stall = std::fabs(c.vx) < 1e-9 && std::fabs(c.vy) < 1e-9 &&
                               std::fabs(herr0) < 1.2;
            // With a pedestrian closing head-on, standing still in the opening
            // is the one choice that guarantees the collision: he keeps walking
            // and the dog does not. Clearing the gap means moving out of it,
            // so distance to the target stops mattering for a moment and a full
            // stop is charged instead of being the cheap default.
            const bool pure_stop = std::fabs(c.vx) < 1e-9 && std::fabs(c.vy) < 1e-9 &&
                                   std::fabs(c.wz) < 1e-9;
            const double cost = (ped_ahead ? 0.15 : 1.4) * d2
                                + 0.30 * (1.0 / (mincl + 0.25))
                                - 0.45 * c.vx + 0.22 * std::fabs(c.vy) + 0.08 * std::fabs(c.wz)
                                + (stall ? 0.45 : 0.0)
                                + (ped_ahead && pure_stop ? 1.2 : 0.0);
            if (cost < best_cost) { best_cost = cost; best = c; }
          }
          }
          if (getenv("NAVSIM_DV") && base_z > 1.6 && t > 21.0 && step % 25 == 0)
            fprintf(stderr, "[dv] t=%.1f (%.2f,%.2f,%.2f) yaw=%.2f any=%d cost=%.2f tgt=(%.2f,%.2f) dgoal=%.2f\n",
                    t, x, y, base_z, yaw, any ? 1 : 0, best_cost, tx, ty, dgoal);
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
          // Only the event log is left here: what the local planner did about
          // him (stop, shuffle sideways, back off) has already been decided.
          {
            const bool stopped = (vx == 0.0 && vy == 0.0 && wz == 0.0);
            if (ped_ahead && stopped &&
                (out.events.empty() || t - out.events.back().first > 1.2)) {
              out.events.push_back({t, "前方有行人，局部规划选择停等"});
            }
          }
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

    // Standing height comes from the surfaces themselves, never from the
    // route. The route knows the whole flight, which made the body jump from
    // the top of run A straight onto the second-floor slab and skip run B; and
    // it is absent exactly when A* cannot plan (which is most of a flight), so
    // gating the height on have_plan left the dog at the landing for ever.
    //
    // Probe under the feet and one riser ahead: the torso rides 0.24-0.44 m
    // above the standing face, so a 0.02 m lift per tick lets it step up onto
    // the next tread without the band ever reading that tread as a wall.
    {
      double under = -1e9;
      const double cyw = std::cos(yaw), syw = std::sin(yaw);
      for (double f : {-0.10, 0.20, 0.45}) {
        const double qx = x + f * cyw, qy = y + f * syw;
        for (const auto& o : sc.obs) {
          if (o.kind != 0) continue;
          if (qx < o.a[0] || qx > o.a[2] || qy < o.a[1] || qy > o.a[3]) continue;
          // Only surfaces within one riser of where we stand: anything higher
          // is a wall or the floor above, not the next step.
          if (o.z1 > base_z + 0.26) continue;
          under = std::max(under, o.z1);
        }
      }
      if (under > -1e8) {
        const double face = under + 0.02;   // cell centre, not the surface
        base_z += std::max(-0.02, std::min(0.02, face - base_z));
      }
    }

    // Metering is per tick; only the trace is sampled, so the reported
    // distance and clearance are never aliased by the sample rate.
    {
      const double c = clearance(obs, t, x, y, base_z);
      out.min_clear = std::min(out.min_clear, c);
    }
    out.length += std::hypot(x - px_prev, y - py_prev);
    px_prev = x; py_prev = y;

    if (step % 5 == 0) {  // 0.1 s trace sample
      Sample s;
      s.t = t; s.x = x; s.y = y; s.z = base_z; s.yaw = yaw;
      s.vx = vx; s.vy = vy; s.wz = wz;
      s.hit = clearance(obs, t, x, y, base_z) < 0.02;
      out.trace.push_back(s);
    }

    if (getenv("NAVSIM_COL") && base_z > 3.0 && base_z < 3.5 && step % 25 == 0) {
      int ix, iy, iz0;
      if (grid.worldToIndex(x, y, base_z + 0.24, ix, iy, iz0)) {
        fprintf(stderr, "[col] t=%.1f (%.2f,%.2f,%.2f) ahead:", t, x, y, base_z);
        for (int gx = 0; gx >= -4; --gx) {
          int ax, ay, az;
          const double qx = x - 0.10 * (-gx);
          if (!grid.worldToIndex(qx, y, base_z + 0.24, ax, ay, az)) continue;
          fprintf(stderr, "  x%.2f z%.2f:", qx,
                  grid.originZ() + (az - 3 + 0.5) * grid.resolution());
          for (int d = -3; d <= 20; ++d)
            fprintf(stderr, "%d", static_cast<int>(grid.at(ax, ay, az + d)));
        }
        fprintf(stderr, "\n    truth at (%.2f,%.2f):", x - 0.6, y);
        for (const auto& o : sc.obs) {
          if (o.kind != 0) continue;
          if (x - 0.6 < o.a[0] || x - 0.6 > o.a[2] || y < o.a[1] || y > o.a[3]) continue;
          fprintf(stderr, " [%.2f-%.2f]", o.z0, o.z1);
        }
        fprintf(stderr, "\n");
      }
    }
    if (hitsTruth(obs, t, x, y, base_z, yaw)) {
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
  o << "\"start_z\":" << sc.sz << ",\"goal_z\":" << sc.gz << ",";
  o << "\"z_range\":[" << sc.zmin << "," << sc.zmax << "],";

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
    std::snprintf(buf, sizeof(buf), "[%.3f,%.3f,%.3f,%.3f]", s.x, s.y, s.z, s.yaw);
    o << buf;
  }
  o << "],";

  o << "\"global_path\":[";
  for (size_t i = 0; i < r.global_path.size(); ++i) {
    if (i) o << ",";
    char buf[48];
    std::snprintf(buf, sizeof(buf), "[%.3f,%.3f,%.3f]",
                  r.global_path[i].x, r.global_path[i].y, r.global_path[i].z);
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
    o << "\"" << r.events[i].second << "\"";
  }
  o << "]";
  {
    o << ",\"local_costmap\":{\"r\":2.0,\"res\":0.10,\"frames\":[";
    for (size_t i = 0; i < r.wins.size(); ++i) {
      if (i) o << ",";
      const auto& w = r.wins[i];
      o << "{\"t\":" << w.t << ",\"cx\":" << w.cx << ",\"cy\":" << w.cy << ",\"cells\":[";
      for (size_t j = 0; j < w.cells.size(); ++j) {
        if (j) o << ",";
        o << "[" << w.cells[j][0] << "," << w.cells[j][1] << "]";
      }
      o << "]}";
    }
    o << "]}";
  }
  if (r.probe_t > 0) {
    o << ",\"map_probe\":{\"t\":" << r.probe_t << ",\"y\":" << r.probe_y
      << ",\"x0\":2.70,\"x_step\":0.10,\"z0\":4.30,\"z_step\":-0.10,\"cells\":[";
    for (size_t i = 0; i < r.probe_cells.size(); ++i) {
      if (i) o << ",";
      o << "[";
      for (size_t j = 0; j < r.probe_cells[i].size(); ++j) {
        if (j) o << ",";
        o << r.probe_cells[i][j];
      }
      o << "]";
    }
    o << "]}";
  }
  o << "}";
  return o.str();
}

}  // namespace

int main() {
  // fourFloor() is back in: the report now carries an X-Z section, which is the
  // one view that shows where a four-storey run actually stops.
  // Both local planners on every scene, so the deployed one and the simulator's
  // own can be compared on identical geometry. NeuPAN is the default.
  std::vector<Scenario> scenarios;
  for (const char* who : {"neupan", "dwa"}) {
    for (Scenario base : {officeLoop(), corridorBlock(), narrowGate(),
                          stairTwoFloor(), fourFloor()}) {
      base.planner = who;
      const std::string tag = (std::string(who) == "neupan") ? "NeuPAN" : "DWA";
      base.name += " · " + tag;
      scenarios.push_back(base);
    }
  }
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
