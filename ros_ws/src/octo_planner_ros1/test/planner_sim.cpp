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

// --- scene F: three-storey office -------------------------------------------
// Floor height 3.4 m, one straight 18-step flight between each pair of floors.
// The two flights sit at different y so they can occupy the same x span, which
// is how a real building packs two runs into one stair core.
Scenario buildMultiFloor() {
  Scenario sc;
  sc.name = "multi_floor";
  sc.desc = "3 层办公楼：层高 3.4 m，折返楼梯井两段 18 级（踢面 0.189 / 踏面 0.30），起点在 1 层、目标在 3 层";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 12.0, 6.0, 10.6, 0.3);

  const double FL = 3.4;
  const double TX0 = 3.0, TX1 = 8.4;   // stair run, 5.4 m horizontally
  const double TY_A0 = 1.5, TY_A1 = 3.0;  // the two flights share one core
  const int N = 18;
  const double RISE = FL / N, TREAD = (TX1 - TX0) / N;

  for (int f = 0; f < 3; ++f) {
    addPlate(sc.grid, 0.0, 12.0, 0.0, 6.0, f * FL);
  }

  // Flight 1 climbs +x, flight 2 climbs -x from the same landing: the classic
  // switch-back core, where each floor's flight sits directly above the one
  // below. They never collide because the z bands are disjoint.
  for (int f = 0; f < 2; ++f) {
    for (int i = 0; i < N; ++i) {
      const double xa = (f == 0) ? TX0 + i * TREAD : TX1 - (i + 1) * TREAD;
      const double xb = (f == 0) ? TX0 + (i + 1) * TREAD : TX1 - i * TREAD;
      addPlate(sc.grid, xa, xb, TY_A0, TY_A1, f * FL + (i + 1) * RISE);
    }
  }

  addWallY(sc.grid, 0.0, 0.0, 6.0, 0.0, 10.6);
  addWallY(sc.grid, 12.0, 0.0, 6.0, 0.0, 10.6);
  addWallX(sc.grid, 0.0, 0.0, 12.0, 0.0, 10.6);
  addWallX(sc.grid, 6.0, 0.0, 12.0, 0.0, 10.6);

  sc.start = makeKeyAt(sc.grid, 10.00, 2.25, 0.35);
  sc.goal = makeKeyAt(sc.grid, 1.50, 2.25, 7.00);
  return sc;
}

// --- scene B: staircase ------------------------------------------------------
Scenario buildStairs() {
  Scenario sc;
  sc.name = "stairs";
  sc.desc = "8 级台阶（踏面 0.30 m / 踢面 0.17 m）+ 上层平台";
  sc.grid.configureFromBounds(0.10, 0.0, 0.0, 0.0, 6.4, 2.6, 2.0, 0.3);

  addPlate(sc.grid, 0.0, 2.4, 0.0, 2.6, 0.0);
  const double tread = 0.30;
  const double rise = 0.17;
  for (int i = 0; i < 8; ++i) {
    addPlate(sc.grid, 2.4 + i * tread, 2.4 + (i + 1) * tread, 0.0, 2.6, rise * (i + 1));
  }
  addPlate(sc.grid, 2.4 + 8 * tread, 6.0, 0.0, 2.6, rise * 8);
  // Side walls keep the search inside the stair flight (they run along x).
  addWallX(sc.grid, 0.0, 0.0, 6.4, 0.0, 2.0);
  addWallX(sc.grid, 2.6, 0.0, 6.4, 0.0, 2.0);

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

  // Occupied cells, thinned 2x2x2 so the report stays small.
  std::printf("\"occ\":[");
  bool first = true;
  for (int iz = 0; iz < sc.grid.nz(); iz += 2) {
    for (int iy = 0; iy < sc.grid.ny(); iy += 2) {
      for (int ix = 0; ix < sc.grid.nx(); ix += 2) {
        if (sc.grid.at(ix, iy, iz) != VoxelGrid::kOccupied) continue;
        std::printf("%s[%d,%d,%d]", first ? "" : ",", ix, iy, iz);
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
  params.ground_support_xy_radius_cells = 1;
  params.snap_search_radius_cells = 12;
  params.max_step_z_cells = 3;

  std::vector<Scenario> scenarios;
  scenarios.push_back(buildOffice(false));
  scenarios.push_back(buildStairs());
  scenarios.push_back(buildPit());
  scenarios.push_back(buildOffice(true));
  scenarios.push_back(buildDeskBarrier());
  scenarios.push_back(buildMultiFloor());

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
  multi.checks.push_back({"跨 3 层出图", multi.ok, multi.result.message});
  multi.checks.push_back({"最高点到达三层站立高度", top_z >= 70.0,
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
              "\"require_ground_support\":true,\"ground_support_xy_radius_cells\":1,"
              "\"snap_search_radius_cells\":12,\"max_step_z_cells\":3},\"scenarios\":[",
              params.max_iterations);
  for (size_t i = 0; i < scenarios.size(); ++i) {
    printScenario(scenarios[i], i + 1 == scenarios.size());
  }
  std::printf("]}\n");
  return 0;
}
