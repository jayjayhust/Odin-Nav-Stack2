// Copyright 2026 Manifold Tech Ltd.
// 3D A* over VoxelGrid, ROS1 port of jie_3d_nav/octo_planner.
#ifndef OCTO_PLANNER_ROS1_ASTAR3D_H
#define OCTO_PLANNER_ROS1_ASTAR3D_H

#include <string>
#include <vector>

#include "octo_planner_ros1/voxel_grid.h"

namespace octo_planner_ros1 {

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

class AStar3D {
 public:
  struct Params {
    int max_iterations = 500000;
    bool require_ground_support = true;
    int ground_support_xy_radius_cells = 0;
    int snap_search_radius_cells = 12;
    bool allow_diagonal = true;
    // A single expansion may climb at most this many cells in z. Keeps the
    // search from teleporting between floors through an unobserved shaft.
    int max_step_z_cells = 3;
    // Extra cost per metre away from the goal's z plane, so that the planner
    // prefers the floor it is asked to stay on when a staircase branch exists.
    double z_bias = 0.0;
  };

  struct Result {
    bool ok = false;
    std::string message;
    std::vector<VoxelKey> path;  // includes start and goal
    int expanded = 0;
    // Order in which cells were closed, used by the offline simulator to
    // animate the search. Kept separate from the planner's own data flow.
    std::vector<VoxelKey> trace;
  };

  Result plan(const VoxelGrid& grid,
              const VoxelKey& start,
              const VoxelKey& goal,
              const Params& params,
              bool record_trace = false) const;
};

}  // namespace octo_planner_ros1

#endif  // OCTO_PLANNER_ROS1_ASTAR3D_H
