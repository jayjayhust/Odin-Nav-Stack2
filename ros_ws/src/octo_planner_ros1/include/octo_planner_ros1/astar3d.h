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
    // Treat unobserved cells as traversable. A stair core is mostly unobserved
    // from the foot of a flight: sensor shadows leave vertical bands of unknown
    // where the risers ahead and the flight overhead block the ray fan, and
    // demanding a FREE cell makes that flight unplannable while every tread in
    // it is mapped correctly. Off by default - flat scenes keep the strict rule.
    bool allow_unknown_cells = false;

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

    // A stair run the path must stay centred on.
    //
    // Why this exists: the stair segment is not tracked by the local planner.
    // The floor transition hook stops NeuPAN, switches the climbing gait and
    // drives straight open loop for a fixed time. So whatever lateral offset
    // the global path has when the robot reaches the foot of the flight is
    // carried, uncorrected, all the way up. Plain step cost has no idea about
    // any of that and returns the shortest route, which hugs a tread edge -
    // correct as A*, wrong as a plan.
    //
    // A corridor adds `weight` extra cost per metre of lateral distance from
    // y_centre, inside a box in world coordinates. It only reshapes the route
    // inside the box; open floors are untouched.
    struct Corridor {
      double x_min = 0.0;   // metres
      double x_max = 0.0;   // metres
      double y_centre = 0.0;  // metres, the line to stay on
      double y_half = 0.0;    // metres, half-width of the box
      double z_min = 0.0;   // metres
      double z_max = 0.0;   // metres
      double weight = 0.0;  // extra cost per metre off centre
    };
    std::vector<Corridor> corridors;
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
