// Copyright 2026 Manifold Tech Ltd.
// ROS1 port of the 3D occupancy planner used by jie_3d_nav/octo_planner.
#ifndef OCTO_PLANNER_ROS1_VOXEL_GRID_H
#define OCTO_PLANNER_ROS1_VOXEL_GRID_H

#include <cstdint>
#include <vector>

namespace octo_planner_ros1 {

// Sparse-free 3D occupancy grid.
//
// The layout mirrors an octomap::OcTree queried at a fixed resolution: every
// cell is UNKNOWN until a surface point is cast into it, the column above a
// surface point is cleared as FREE (that is what makes the space walkable),
// and a separate inflation pass turns FREE cells too close to a surface into
// INFLATED so the planner never routes the robot body through them.
class VoxelGrid {
 public:
  static constexpr uint8_t kUnknown = 0;
  static constexpr uint8_t kFree = 1;
  static constexpr uint8_t kOccupied = 2;
  static constexpr uint8_t kInflated = 3;

  VoxelGrid() = default;

  void configure(double resolution,
                 double origin_x, double origin_y, double origin_z,
                 int nx, int ny, int nz);
  void configureFromBounds(double resolution,
                           double min_x, double min_y, double min_z,
                           double max_x, double max_y, double max_z,
                           double margin_m);
  void clear();

  bool empty() const { return data_.empty(); }
  double resolution() const { return res_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  double originX() const { return ox_; }
  double originY() const { return oy_; }
  double originZ() const { return oz_; }
  int cellCount() const { return static_cast<int>(data_.size()); }

  bool inRange(int ix, int iy, int iz) const;
  uint8_t at(int ix, int iy, int iz) const;
  void set(int ix, int iy, int iz, uint8_t value);

  bool worldToIndex(double x, double y, double z,
                    int& ix, int& iy, int& iz) const;
  void indexToWorld(int ix, int iy, int iz,
                    double& x, double& y, double& z) const;

  // Cast one surface point. The cell itself becomes OCCUPIED; the column
  // directly above it (clearance_cells high) becomes FREE.
  void insertSurfacePoint(double x, double y, double z, int clearance_cells);

  // Grow every occupied cell by (xy_radius, z_radius) cells. FREE cells that
  // fall inside become INFLATED; UNKNOWN is left alone so the robot may still
  // plan through space that was simply never observed.
  //
  // skip_standing_surfaces: do not inflate a cell whose directly upper
  // neighbour is FREE. Such a cell is a floor / tread / tabletop, i.e. a
  // surface the robot walks ON, not an obstacle it would collide with.
  // Without this flag a 0.17 m rise makes the upper tread's surface and the
  // lower tread's standing cell share a z layer, and the radius inflation
  // then erases the whole staircase.
  void inflate(int xy_radius_cells, int z_radius_cells,
               bool skip_standing_surfaces = false);

  // Undo the inflation pass: every INFLATED cell goes back to FREE. Lets the
  // planner re-inflate from the raw surface every request without keeping a
  // second copy of the grid.
  void deinflate();

  // True if there is a supporting surface within xy_radius cells one step
  // below (iz - 1). This is the ROS1 equivalent of octo_planner's
  // `require_ground_support` / `ground_support_xy_radius_cells`.
  bool hasGroundSupport(int ix, int iy, int iz, int xy_radius_cells) const;

  // Highest supporting surface inside column (ix, iy), searching downward
  // from iz_hint. Returns -1 when the column holds no surface.
  int groundBelow(int ix, int iy, int iz_hint) const;

  // Nearest traversable cell to (ix,iy,iz) inside a cubic window, or false.
  // allow_inflated: start and goal often sit inside the robot-radius band
    // (against a wall, at a stair head); accepting INFLATED there keeps the
    // planner usable without weakening the constraint on intermediate cells.
    bool nearestWalkable(int ix, int iy, int iz, int radius_cells,
                         int& out_x, int& out_y, int& out_z,
                         bool allow_inflated = false) const;

 private:
  int index(int ix, int iy, int iz) const {
    return (iz * ny_ + iy) * nx_ + ix;
  }

  double res_ = 0.1;
  double ox_ = 0.0, oy_ = 0.0, oz_ = 0.0;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<uint8_t> data_;
};

}  // namespace octo_planner_ros1

#endif  // OCTO_PLANNER_ROS1_VOXEL_GRID_H
