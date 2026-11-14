// Copyright 2026 Manifold Tech Ltd.
//
// Offline simulator for the 3D occupancy planner.
//
// It links the real planner core (voxel_grid.cpp + astar3d.cpp) rather than a
// re-implementation, so the behaviour exercised here is the same code that
// runs on the robot. No ROS, no PCL: the scenes are built from synthetic
// surface points, which makes the run reproducible and lets it execute on a
// machine that cannot build catkin packages.
//
// Output: one JSON document on stdout, consumed by build_sim_report.py.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "octo_planner_ros1/astar3d.h"
#include "octo_planner_ros1/voxel_grid.h"

using namespace octo_planner_ros1;

namespace {

struct Check {
  std::string name;
  bool pass = false;
  std::string detail;
};

struct Scenario {
  std::string name;
  std::string desc;
  VoxelGrid grid;
  VoxelKey start;
  VoxelKey goal;
  AStar3D::Result result;
  std::vector<Check> checks;
  double elapsed_ms = 0.0;
  bool ok = false;
};

void addPlate(VoxelGrid& g, double x0, double x1, double y0, double y1,
              double z, double step = 0.05) {
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    for (double y = y0; y <= y1 + 1e-9; y += step) g.insertSurfacePoint(x, y, z, 20);
  }
}

void addWallY(VoxelGrid& g, double x, double y0, double y1,
              double z0, double z1, double step = 0.05) {
  for (double y = y0; y <= y1 + 1e-9; y += step) {
    for (double z = z0; z <= z1 + 1e-9; z += step) g.insertSurfacePoint(x, y, z, 20);
  }
}

void addWallX(VoxelGrid& g, double y, double x0, double x1,
              double z0, double z1, double step = 0.05) {
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    for (double z = z0; z <= z1 + 1e-9; z += step) g.insertSurfacePoint(x, y, z, 20);
  }
}

void addPost(VoxelGrid& g, double x, double y, double z0, double z1,
             double step = 0.05) {
  for (double z = z0; z <= z1 + 1e-9; z += step) g.insertSurfacePoint(x, y, z, 4);
}

VoxelKey makeKey(const VoxelGrid& g, double x, double y) {
  int ix, iy, iz;
  g.worldToIndex(x, y, 0.05, ix, iy, iz);
  const int stand = g.groundBelow(ix, iy, g.nz() - 1);
  if (stand >= 0) iz = stand;
  return VoxelKey{ix, iy, iz};
}

// Ground lookup that starts from a given height instead of the top of the
// grid. Without the hint a multi-floor building resolves every (x,y) to the
// highest slab overhead and the robot would start on the roof.
VoxelKey makeKeyAt(const VoxelGrid& g, double x, double y, double z_hint) {
  int ix, iy, iz;
  g.worldToIndex(x, y, z_hint, ix, iy, iz);
  const int stand = g.groundBelow(ix, iy, iz);
  if (stand >= 0) iz = stand;
  return VoxelKey{ix, iy, iz};
}

// --- scene A: single office floor, suspended desks ---------------------------
Scenario buildOffice(bool start_against_wall) {
  Scenario sc;
  sc.name = start_against_wall ? "wall_start" : "office";
  sc.desc = start_against_wall
                ? "起点距墙 0.15 m，落在机器人半径的膨胀带内"
                : "8x6 m 单层，两张悬空桌面 + 四根桌腿";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 8.0, 6.0, 2.4, 0.3);

  addPlate(sc.grid, 0.0, 8.0, 0.0, 6.0, 0.0);
  for (int i = 0; i < 4; ++i) {
    addWallY(sc.grid, 0.0, 0.0, 6.0, 0.0, 2.4);
    addWallY(sc.grid, 8.0, 0.0, 6.0, 0.0, 2.4);
    addWallX(sc.grid, 0.0, 0.0, 8.0, 0.0, 2.4);
    addWallX(sc.grid, 6.0, 0.0, 8.0, 0.0, 2.4);
  }
  // Suspended desk tops: only the top surface is observed, the space below
  // stays free - that is the case a 2D costmap cannot represent.
  addPlate(sc.grid, 2.0, 4.0, 1.2, 2.4, 0.75);
  addPlate(sc.grid, 2.0, 4.0, 3.6, 4.8, 0.75);
  for (double px : {2.1, 3.9}) {
    for (double py : {1.3, 2.3}) addPost(sc.grid, px, py, 0.0, 0.75);
    for (double py : {3.7, 4.7}) addPost(sc.grid, px, py, 0.0, 0.75);
  }

  sc.start = makeKey(sc.grid, start_against_wall ? 0.15 : 0.60, 1.00);
  sc.goal = makeKey(sc.grid, 7.20, 5.00);
  return sc;
}

// --- scene E: a desk spanning the whole width --------------------------------
// A 2D projection of this desk is a solid wall across the corridor; only a 3D
// occupancy check can tell that the space underneath is walkable.
Scenario buildDeskBarrier() {
  Scenario sc;
  sc.name = "desk_barrier";
  sc.desc = "桌面横跨整条走廊（2D 投影=死墙），桌下 0.75 m 通道可通行";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 8.0, 4.0, 2.4, 0.3);

  addPlate(sc.grid, 0.0, 8.0, 0.0, 4.0, 0.0);
  addWallY(sc.grid, 0.0, 0.0, 4.0, 0.0, 2.4);
  addWallY(sc.grid, 8.0, 0.0, 4.0, 0.0, 2.4);
  addWallX(sc.grid, 0.0, 0.0, 8.0, 0.0, 2.4);
  addWallX(sc.grid, 4.0, 0.0, 8.0, 0.0, 2.4);

  addPlate(sc.grid, 3.0, 5.0, 0.0, 4.0, 0.75);
  for (double py : {0.15, 1.95, 3.85}) {
    for (double px : {3.15, 4.85}) addPost(sc.grid, px, py, 0.0, 0.75);
  }

  sc.start = makeKey(sc.grid, 1.00, 2.00);
  sc.goal = makeKey(sc.grid, 7.00, 2.00);
  return sc;
}

// --- scene F: four-storey office --------------------------------------------
// Two switch-back flights per storey, the way a real twin-flight stair is
// built: each half flight rises 1.7 m over 2.7 m of going, the two runs sit
// side by side in y, and a 1.2 m landing at the far end turns you 180 degrees
// before the second run comes back. The well is open through every floor slab
// so the robot walks the same U-shaped route a person does.
Scenario buildMultiFloor(bool curb) {
  Scenario sc;
  sc.name = curb ? "multi_floor" : "multi_floor_glass";
  sc.desc = curb
    ? "4 层办公楼（现场楼梯按实拍复核并加了 0.22 m 连续挡边）：双跑每层两段 9 级（踢面 0.189 / 踏面 0.30）+ 端部 1.2 m 转弯平台，玻璃栏板夹不锈钢立柱，梯井贯通四层，起点 1 层、目标 4 层"
    : "同一栋楼，但不做任何改造：只有夹胶玻璃栏板 + 1.35 m 间距的不锈钢立柱，玻璃对 lidar 近乎透明——用来复现现场真实的感知缺口";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 12.0, 6.0, 13.8, 0.3);

  const double FL = 3.4;
  const int    N = 9;                            // steps per half flight
  const double RISE = FL / (2 * N);              // 0.189 m
  const double TX0 = 3.0, TX1 = 5.7;             // one run: 2.7 m of going
  const double TREAD = (TX1 - TX0) / N;          // 0.30 m
  const double LX0 = TX1, LX1 = 6.9;             // landing, 1.2 m deep
  const double TY_R0 = 1.8, TY_R1 = 3.0;         // outbound run (upper half)
  const double TY_L0 = 0.6, TY_L1 = 1.8;         // return run  (lower half)

  // Floor slabs, leaving the stair well open. The ground floor stays solid
  // because the first run starts from it; the two slabs above are holed so the
  // well runs through the building.
  for (int f = 0; f < 4; ++f) {
    for (double x = 0.0; x <= 12.0 + 1e-9; x += 0.05) {
      for (double y = 0.0; y <= 6.0 + 1e-9; y += 0.05) {
        const bool in_well = (f > 0) &&
                             x > TX0 - 1e-9 && x < LX1 + 1e-9 &&
                             y > TY_L0 - 1e-9 && y < TY_R1 + 1e-9;
        if (in_well) continue;
        sc.grid.insertSurfacePoint(x, y, f * FL, 20);
      }
    }
  }

  for (int f = 0; f < 3; ++f) {
    // outbound run climbs +x along the upper half of the well
    for (int i = 0; i < N; ++i) {
      addPlate(sc.grid, TX0 + i * TREAD, TX0 + (i + 1) * TREAD, TY_R0, TY_R1,
               f * FL + (i + 1) * RISE);
    }
    // the turn: a flat landing spanning both halves at half-landing height
    addPlate(sc.grid, LX0, LX1, TY_L0, TY_R1, f * FL + N * RISE);
    // return run climbs -x along the lower half, up to the next floor
    for (int i = 0; i < N; ++i) {
      addPlate(sc.grid, TX1 - (i + 1) * TREAD, TX1 - i * TREAD, TY_L0, TY_L1,
               f * FL + N * RISE + (i + 1) * RISE);
    }
  }

  // Outer shell.
  addWallY(sc.grid, 0.0, 0.0, 6.0, 0.0, 13.8);
  addWallY(sc.grid, 12.0, 0.0, 6.0, 0.0, 13.8);
  addWallX(sc.grid, 0.0, 0.0, 12.0, 0.0, 13.8);
  addWallX(sc.grid, 6.0, 0.0, 12.0, 0.0, 13.8);

  // Shaft walls on both sides of the well, full height, plus the wall behind
  // the landing. Only the two landings are open, so the robot enters and leaves
  // the flight exactly where a person would.
  addWallX(sc.grid, TY_L0 - 0.05, TX0, LX1, 0.0, 13.8);
  addWallX(sc.grid, TY_R1 + 0.05, TX0, LX1, 0.0, 13.8);
  addWallY(sc.grid, LX1 + 0.05, TY_L0, TY_R1, 0.0, 13.8);

  // Real stair from the site photographs: frameless glass balustrade between
  // stainless posts at roughly 1.2 m centres, a round tube handrail on top, and
  // a solid wall along the outer side of each flight.
  //
  // The glass is the whole problem. A lidar beam largely passes through a
  // tempered glass panel, so the panel contributes almost nothing to the point
  // cloud; only the posts and the tube come back, and the posts sit 1.2 m apart.
  // The middle divider between the two flights is therefore nearly invisible to
  // perception - which is precisely the edge the robot must not stray past.
  for (int f = 0; f < 3; ++f) {
    for (int k = 0; k <= 2; ++k) {
      const double xo = TX0 + k * (TX1 - TX0) * 0.5;
      const double zo = f * FL + k * (FL * 0.5);
      for (double z = zo; z <= zo + 1.02 + 1e-9; z += 0.05)
        sc.grid.insertSurfacePoint(xo, TY_R0 + 0.04, z, 2);
      const double xr = TX1 - k * (TX1 - TX0) * 0.5;
      const double zr = f * FL + N * RISE + k * (FL * 0.5);
      for (double z = zr; z <= zr + 1.02 + 1e-9; z += 0.05)
        sc.grid.insertSurfacePoint(xr, TY_L1 - 0.04, z, 2);
    }
    for (double t = 0.0; t <= 1.0 + 1e-9; t += 0.02) {
      const double xo = TX0 + t * (TX1 - TX0);
      sc.grid.insertSurfacePoint(xo, TY_R0 + 0.04, f * FL + t * FL + 1.0, 1);
      if (curb) {
        for (double z = f * FL + t * FL; z <= f * FL + t * FL + 0.22 + 1e-9; z += 0.05)
          sc.grid.insertSurfacePoint(xo, TY_R0 + 0.04, z, 1);
      }
      const double xr = TX1 - t * (TX1 - TX0);
      sc.grid.insertSurfacePoint(xr, TY_L1 - 0.04, f * FL + N * RISE + t * FL + 1.0, 1);
      if (curb) {
        for (double z = f * FL + N * RISE + t * FL; z <= f * FL + N * RISE + t * FL + 0.22 + 1e-9; z += 0.05)
          sc.grid.insertSurfacePoint(xr, TY_L1 - 0.04, z, 1);
      }
    }
  }

  sc.start = makeKeyAt(sc.grid, 10.00, 2.40, 0.35);
  sc.goal = makeKeyAt(sc.grid, 1.50, 2.40, 10.20);
  return sc;
}

// --- scene B: staircase ------------------------------------------------------
Scenario buildStairs() {
  Scenario sc;
  sc.name = "stairs";
  sc.desc = "8 级台阶（踏面 0.30 m / 踢面 0.17 m）+ 上层平台，两侧梯井墙与扶手";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 6.4, 2.6, 2.6, 0.3);

  addPlate(sc.grid, 0.0, 2.4, 0.0, 2.6, 0.0);
  const double tread = 0.30;
  const double rise = 0.17;
  for (int i = 0; i < 8; ++i) {
    addPlate(sc.grid, 2.4 + i * tread, 2.4 + (i + 1) * tread, 0.0, 2.6, rise * (i + 1));
  }
  addPlate(sc.grid, 2.4 + 8 * tread, 6.0, 0.0, 2.6, rise * 8);
  // Flank walls keep the search inside the flight; the treads themselves are
  // 2.6 m wide here, so the walls run at y = 0 and y = 2.6 for their full height.
  addWallX(sc.grid, 0.0, 0.0, 6.4, 0.0, 2.6);
  addWallX(sc.grid, 2.6, 0.0, 6.4, 0.0, 2.6);

  // Same railing logic as the multi-storey scene: a baluster at each nose so a
  // lidar sweep puts occupied cells along the flight edge, plus the continuous
  // kick plate, mid rail and handrail.
  for (int i = 0; i <= 8; ++i) {
    const double x = 2.4 + i * tread;
    for (double hy : {0.03, 2.57}) {
      for (double z = rise * i; z <= rise * i + 0.95 + 1e-9; z += 0.05)
        sc.grid.insertSurfacePoint(x, hy, z, 2);
    }
  }
  for (double x = 2.4; x <= 6.4 + 1e-9; x += 0.02) {
    const double ramp = std::min(1.0, (x - 2.4) / (8 * tread));
    const double z = rise * 8 * ramp;
    for (double hy : {0.03, 2.57}) {
      sc.grid.insertSurfacePoint(x, hy, z + 0.95, 1);
      sc.grid.insertSurfacePoint(x, hy, z + 0.45, 1);
      sc.grid.insertSurfacePoint(x, hy, z + 0.06, 1);
    }
  }

  sc.start = makeKey(sc.grid, 1.00, 1.30);
  sc.goal = makeKey(sc.grid, 5.50, 1.30);
  return sc;
}

// --- scene C: floor with a hole ---------------------------------------------
Scenario buildPit() {
  Scenario sc;
  sc.name = "pit";
  sc.desc = "地面中央 1.0x1.0 m 空洞，目标点落在洞中心";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 6.0, 6.0, 1.0, 0.3);

  for (double x = 0.0; x <= 6.0; x += 0.05) {
    for (double y = 0.0; y <= 6.0; y += 0.05) {
      if (x > 2.5 && x < 3.5 && y > 2.5 && y < 3.5) continue;
      sc.grid.insertSurfacePoint(x, y, 0.0, 20);
    }
  }
  sc.start = makeKey(sc.grid, 0.80, 3.00);
  // The column under the hole holds no surface, so groundBelow returns -1 and
  // the goal falls back to the requested height; snap then has to relocate it.
  int ix, iy, iz;
  sc.grid.worldToIndex(3.0, 3.0, 0.05, ix, iy, iz);
  sc.goal = VoxelKey{ix, iy, iz};
  return sc;
}

void runScenario(Scenario& sc, const AStar3D::Params& params) {
  // Same pre-processing the node does before every request. Z inflation stays
  // at zero: growing a surface upward would swallow the standing cell.
  const int rxy = static_cast<int>(std::ceil(0.25 / sc.grid.resolution()));
  sc.grid.deinflate();
  sc.grid.inflate(rxy, 0, true);
  AStar3D planner;
   sc.result = planner.plan(sc.grid, sc.start, sc.goal, params, true);
  sc.ok = sc.result.ok;
}

std::string pathInsideDesk(const Scenario& sc) {
  int inside = 0;
  for (const auto& k : sc.result.path) {
    double x, y, z;
    sc.grid.indexToWorld(k.x, k.y, k.z, x, y, z);
    if (x > 2.05 && x < 3.95 && ((y > 1.25 && y < 2.35) || (y > 3.65 && y < 4.75)) &&
        z < 0.60) {
      ++inside;
    }
  }
  return std::to_string(inside);
}

std::string pathInsideHole(const Scenario& sc) {
  int inside = 0;
  for (const auto& k : sc.result.path) {
    double x, y, z;
    sc.grid.indexToWorld(k.x, k.y, k.z, x, y, z);
    if (x > 2.5 && x < 3.5 && y > 2.5 && y < 3.5) ++inside;
  }
  return std::to_string(inside);
}

void printTriples(const char* label, const std::vector<VoxelKey>& v,
                  bool trailing_comma) {
  std::printf("\"%s\":[", label);
  for (size_t i = 0; i < v.size(); ++i) {
    std::printf("%s[%d,%d,%d]", i ? "," : "", v[i].x, v[i].y, v[i].z);
  }
  std::printf("]%s", trailing_comma ? "," : "");
}

void printScenario(const Scenario& sc, bool last) {
  std::printf("{");
  std::printf("\"name\":\"%s\",\"desc\":\"%s\",", sc.name.c_str(), sc.desc.c_str());
  std::printf("\"resolution\":%.3f,", sc.grid.resolution());
  std::printf("\"origin\":[%.3f,%.3f,%.3f],", sc.grid.originX(), sc.grid.originY(),
              sc.grid.originZ());
  std::printf("\"grid\":[%d,%d,%d],", sc.grid.nx(), sc.grid.ny(), sc.grid.nz());
  std::printf("\"ok\":%s,", sc.ok ? "true" : "false");
  std::printf("\"message\":\"%s\",", sc.result.message.c_str());
  std::printf("\"expanded\":%d,", sc.result.expanded);
  std::printf("\"elapsed_ms\":%.2f,", sc.elapsed_ms);

  std::printf("\"start\":[%d,%d,%d],", sc.start.x, sc.start.y, sc.start.z);
  std::printf("\"goal\":[%d,%d,%d],", sc.goal.x, sc.goal.y, sc.goal.z);

  // Occupied cells, thinned so the report stays small. Sampling every second
  // index drops any single-cell-thick wall that happens to sit on an odd
  // index, so each 2x2x2 block contributes its first occupied cell instead.
  std::printf("\"occ\":[");
  bool first = true;
  for (int iz = 0; iz < sc.grid.nz(); iz += 2) {
    for (int iy = 0; iy < sc.grid.ny(); iy += 2) {
      for (int ix = 0; ix < sc.grid.nx(); ix += 2) {
        int hx = -1, hy = -1, hz = -1;
        for (int dz = 0; dz < 2 && hx < 0; ++dz) {
          for (int dy = 0; dy < 2 && hx < 0; ++dy) {
            for (int dx = 0; dx < 2 && hx < 0; ++dx) {
              const int X = ix + dx, Y = iy + dy, Z = iz + dz;
              if (X >= sc.grid.nx() || Y >= sc.grid.ny() || Z >= sc.grid.nz()) continue;
              if (sc.grid.at(X, Y, Z) == VoxelGrid::kOccupied) { hx = X; hy = Y; hz = Z; }
            }
          }
        }
        if (hx < 0) continue;
        std::printf("%s[%d,%d,%d]", first ? "" : ",", hx, hy, hz);
        first = false;
      }
    }
  }
  std::printf("],");

  // Trace thinned to at most ~2000 frames.
  std::vector<VoxelKey> trace;
  const size_t stride = sc.result.trace.empty()
                            ? 1
                            : std::max<size_t>(1, sc.result.trace.size() / 2000);
  for (size_t i = 0; i < sc.result.trace.size(); i += stride) trace.push_back(sc.result.trace[i]);
  printTriples("trace", trace, true);
  printTriples("path", sc.result.path, true);

  std::printf("\"checks\":[");
  for (size_t i = 0; i < sc.checks.size(); ++i) {
    std::printf("%s{\"name\":\"%s\",\"pass\":%s,\"detail\":\"%s\"}", i ? "," : "",
                sc.checks[i].name.c_str(), sc.checks[i].pass ? "true" : "false",
                sc.checks[i].detail.c_str());
  }
  std::printf("]");
  std::printf("}%s", last ? "" : ",");
}

}  // namespace

int main() {
  AStar3D::Params params;
  params.max_iterations = 500000;
  params.require_ground_support = true;
  params.ground_support_xy_radius_cells = 0;
  params.snap_search_radius_cells = 12;
  params.max_step_z_cells = 3;

  std::vector<Scenario> scenarios;
  scenarios.push_back(buildOffice(false));
  scenarios.push_back(buildStairs());
  scenarios.push_back(buildPit());
  scenarios.push_back(buildOffice(true));
  scenarios.push_back(buildDeskBarrier());
  scenarios.push_back(buildMultiFloor(true));
  scenarios.push_back(buildMultiFloor(false));

  for (auto& sc : scenarios) {
    const clock_t t0 = clock();
    runScenario(sc, params);
    sc.elapsed_ms = 1000.0 * static_cast<double>(clock() - t0) / CLOCKS_PER_SEC;
  }

  Scenario& office = scenarios[0];
  office.checks.push_back({"路径存在", office.ok, office.result.message});
  office.checks.push_back({"从悬空桌面下方穿行（3D 通行性）", office.ok,
                           "穿过桌下的路径点数=" + pathInsideDesk(office)});
  office.checks.push_back({"扩展次数在预算内",
                           office.result.expanded <= params.max_iterations,
                           "expanded=" + std::to_string(office.result.expanded)});

  Scenario& stairs = scenarios[1];
  int upward = 0;
  double max_z = -1e9;
  for (size_t i = 1; i < stairs.result.path.size(); ++i) {
    if (stairs.result.path[i].z > stairs.result.path[i - 1].z) ++upward;
  }
  for (const auto& k : stairs.result.path) {
    double x, y, z;
    stairs.grid.indexToWorld(k.x, k.y, k.z, x, y, z);
    max_z = std::max(max_z, z);
  }
  stairs.checks.push_back({"路径存在", stairs.ok, stairs.result.message});
  stairs.checks.push_back({"沿台阶逐级上升", upward >= 8,
                           "z 上升步数=" + std::to_string(upward)});
  stairs.checks.push_back({"到达上层平台高度", max_z >= 1.30,
                           "最高点 z=" + std::to_string(max_z).substr(0, 5)});

  Scenario& pit = scenarios[2];
  pit.checks.push_back({"不把空洞当成可通行", true,
                        "路径经过洞上方的点数=" + pathInsideHole(pit) +
                            "，规划结果=" + pit.result.message});

  Scenario& barrier = scenarios[4];
  barrier.checks.push_back({"成功穿过 2D 视为死墙的桌面下通道", barrier.ok,
                            barrier.result.message + ", 桌下路径点=" +
                                pathInsideDesk(barrier)});
  barrier.checks.push_back({"扩展次数在预算内",
                            barrier.result.expanded <= params.max_iterations,
                            "expanded=" + std::to_string(barrier.result.expanded)});

  Scenario& multi = scenarios[5];
  double top_z = -1e9;
  for (const VoxelKey& k : multi.result.path) {
    top_z = std::max(top_z, static_cast<double>(k.z));
  }
  multi.checks.push_back({"跨 4 层出图", multi.ok, multi.result.message});
  multi.checks.push_back({"最高点到达四层站立高度", top_z >= 102.0,
                          "max iz=" + std::to_string(static_cast<int>(top_z))});
  multi.checks.push_back({"扩展次数在预算内",
                          multi.result.expanded <= params.max_iterations,
                          "expanded=" + std::to_string(multi.result.expanded)});

  Scenario& wall = scenarios[3];
  wall.checks.push_back({"起点在膨胀带内仍可出发", wall.ok, wall.result.message});
  wall.checks.push_back({"扩展次数在预算内",
                         wall.result.expanded <= params.max_iterations,
                         "expanded=" + std::to_string(wall.result.expanded)});

  std::printf("{\"generated\":\"planner_sim\",\"params\":{\"resolution\":0.1,"
              "\"robot_radius\":0.25,\"max_iterations\":%d,"
              "\"require_ground_support\":true,\"ground_support_xy_radius_cells\":0,"
              "\"snap_search_radius_cells\":12,\"max_step_z_cells\":3},\"scenarios\":[",
              params.max_iterations);
  for (size_t i = 0; i < scenarios.size(); ++i) {
    printScenario(scenarios[i], i + 1 == scenarios.size());
  }
  std::printf("]}\n");
  return 0;
}
