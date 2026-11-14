// Copyright 2026 Manifold Tech Ltd.
#include "octo_planner_ros1/astar3d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>

namespace octo_planner_ros1 {
namespace {

struct OpenNode {
  float f;
  float g;
  int flat;
  bool operator>(const OpenNode& o) const { return f > o.f; }
};

inline int flatten(const VoxelGrid& g, int ix, int iy, int iz) {
  return (iz * g.ny() + iy) * g.nx() + ix;
}

inline void unflatten(const VoxelGrid& g, int flat, int& ix, int& iy, int& iz) {
  iz = flat / (g.nx() * g.ny());
  const int rem = flat - iz * g.nx() * g.ny();
  iy = rem / g.nx();
  ix = rem - iy * g.nx();
}

inline float heuristic(const VoxelGrid& g, int ax, int ay, int az,
                       int bx, int by, int bz) {
  const float dx = static_cast<float>(ax - bx);
  const float dy = static_cast<float>(ay - by);
  const float dz = static_cast<float>(az - bz);
  return static_cast<float>(g.resolution()) *
         std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

AStar3D::Result AStar3D::plan(const VoxelGrid& grid,
                              const VoxelKey& start_in,
                              const VoxelKey& goal_in,
                              const Params& params,
                              bool record_trace) const {
  Result out;
  if (grid.empty()) {
    out.message = "voxel grid is empty";
    return out;
  }
  if (!grid.inRange(start_in.x, start_in.y, start_in.z) ||
      !grid.inRange(goal_in.x, goal_in.y, goal_in.z)) {
    out.message = "start or goal outside the voxel grid";
    return out;
  }

  VoxelKey start = start_in;
  VoxelKey goal = goal_in;
  const auto usable_endpoint = [](uint8_t v) {
    return v == VoxelGrid::kFree || v == VoxelGrid::kInflated;
  };
  // Prefer snapping onto a genuinely FREE cell: starting inside the
  // robot-radius band leaves every neighbour INFLATED, so the search would
  // expand once and give up. Only fall back to INFLATED when no FREE cell is
  // in range.
  if (grid.at(start.x, start.y, start.z) != VoxelGrid::kFree) {
    VoxelKey probe = start;
    if (!grid.nearestWalkable(probe.x, probe.y, probe.z,
                              params.snap_search_radius_cells,
                              start.x, start.y, start.z, false) &&
        !grid.nearestWalkable(probe.x, probe.y, probe.z,
                              params.snap_search_radius_cells,
                              start.x, start.y, start.z, true)) {
      out.message = "start has no traversable cell within snap radius";
      return out;
    }
  }
  if (grid.at(goal.x, goal.y, goal.z) != VoxelGrid::kFree) {
    VoxelKey probe = goal;
    if (!grid.nearestWalkable(probe.x, probe.y, probe.z,
                              params.snap_search_radius_cells,
                              goal.x, goal.y, goal.z, false) &&
        !grid.nearestWalkable(probe.x, probe.y, probe.z,
                              params.snap_search_radius_cells,
                              goal.x, goal.y, goal.z, true)) {
      out.message = "goal has no traversable cell within snap radius";
      return out;
    }
  }

  const int total = grid.nx() * grid.ny() * grid.nz();
  std::vector<float> g_score(static_cast<size_t>(total),
                             std::numeric_limits<float>::infinity());
  std::vector<int> parent(static_cast<size_t>(total), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(total), 0);

  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open;
  const int start_flat = flatten(grid, start.x, start.y, start.z);
  const int goal_flat = flatten(grid, goal.x, goal.y, goal.z);
  g_score[start_flat] = 0.0f;
  open.push(OpenNode{heuristic(grid, start.x, start.y, start.z, goal.x, goal.y, goal.z),
                     0.0f, start_flat});

  const int max_step_z = std::max(1, params.max_step_z_cells);
  const int reach_goal_cells = 1;
  bool found = false;
  int expanded = 0;

  while (!open.empty()) {
    const OpenNode cur = open.top();
    open.pop();
    if (closed[cur.flat]) continue;
    closed[cur.flat] = 1;
    ++expanded;
    if (expanded > params.max_iterations) {
      out.message = "max_iterations exceeded";
      break;
    }

    int cx, cy, cz;
    unflatten(grid, cur.flat, cx, cy, cz);
    if (record_trace) out.trace.push_back(VoxelKey{cx, cy, cz});

    // Accept a cell adjacent to the goal, not only the exact cell: the goal is
    // usually the projection of a 2D waypoint and is rarely voxel-aligned.
    if (std::abs(cx - goal.x) <= reach_goal_cells &&
        std::abs(cy - goal.y) <= reach_goal_cells &&
        std::abs(cz - goal.z) <= reach_goal_cells) {
      found = true;
      break;
    }

    for (int dz = -max_step_z; dz <= max_step_z; ++dz) {
      const int nz = cz + dz;
      if (nz < 0 || nz >= grid.nz()) continue;
      for (int dy = -1; dy <= 1; ++dy) {
        const int ny = cy + dy;
        if (ny < 0 || ny >= grid.ny()) continue;
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          const int nx = cx + dx;
          if (nx < 0 || nx >= grid.nx()) continue;
          if (!params.allow_diagonal && std::abs(dx) + std::abs(dy) > 1) continue;

          if (grid.at(nx, ny, nz) != VoxelGrid::kFree) continue;
          if (params.require_ground_support &&
              !grid.hasGroundSupport(nx, ny, nz,
                                     params.ground_support_xy_radius_cells)) {
            continue;
          }

          const int nflat = flatten(grid, nx, ny, nz);
          if (closed[nflat]) continue;

          const float step = static_cast<float>(grid.resolution()) *
                             std::sqrt(static_cast<float>(dx * dx + dy * dy + dz * dz));
          const float zbias = params.z_bias > 0.0
                                  ? static_cast<float>(params.z_bias * grid.resolution() *
                                                       std::abs(nz - goal.z))
                                  : 0.0f;
          const float tentative = cur.g + step + zbias;
          if (tentative >= g_score[nflat]) continue;

          g_score[nflat] = tentative;
          parent[nflat] = cur.flat;
          open.push(OpenNode{tentative + heuristic(grid, nx, ny, nz, goal.x, goal.y, goal.z),
                             tentative, nflat});
        }
      }
    }
  }

  out.expanded = expanded;
  if (!found) {
    if (out.message.empty()) out.message = "no path found";
    return out;
  }

  // Reconstruct from the accepted cell back to start.
  int flat = goal_flat;
  if (closed[goal_flat] == 0) {
    // The break happened on an adjacent cell; find it in the closed set.
    bool located = false;
    for (int dz = -reach_goal_cells; dz <= reach_goal_cells && !located; ++dz) {
      for (int dy = -reach_goal_cells; dy <= reach_goal_cells && !located; ++dy) {
        for (int dx = -reach_goal_cells; dx <= reach_goal_cells && !located; ++dx) {
          const int tx = goal.x + dx, ty = goal.y + dy, tz = goal.z + dz;
          if (!grid.inRange(tx, ty, tz)) continue;
          const int tf = flatten(grid, tx, ty, tz);
          if (closed[tf]) {
            flat = tf;
            located = true;
          }
        }
      }
    }
    if (!located) {
      out.message = "goal cell was not closed; reconstruction failed";
      return out;
    }
  }

  std::vector<VoxelKey> reverse_path;
  int guard = 0;
  while (flat >= 0 && guard++ < total) {
    int ix, iy, iz;
    unflatten(grid, flat, ix, iy, iz);
    reverse_path.push_back(VoxelKey{ix, iy, iz});
    if (flat == start_flat) break;
    flat = parent[flat];
  }
  if (reverse_path.empty() || reverse_path.back().x != start.x ||
      reverse_path.back().y != start.y || reverse_path.back().z != start.z) {
    out.message = "path reconstruction did not reach the start cell";
    return out;
  }
  std::reverse(reverse_path.begin(), reverse_path.end());
  out.path = reverse_path;
  out.ok = true;
  out.message = "ok";
  return out;
}

}  // namespace octo_planner_ros1
