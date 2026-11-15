// Copyright 2026 Manifold Tech Ltd.
#include "octo_planner_ros1/voxel_grid.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace octo_planner_ros1 {

// C++14 needs out-of-class definitions for odr-used constexpr members.
constexpr uint8_t VoxelGrid::kUnknown;
constexpr uint8_t VoxelGrid::kFree;
constexpr uint8_t VoxelGrid::kOccupied;
constexpr uint8_t VoxelGrid::kInflated;

void VoxelGrid::configure(double resolution,
                          double origin_x, double origin_y, double origin_z,
                          int nx, int ny, int nz) {
  res_ = resolution > 0.0 ? resolution : 0.1;
  ox_ = origin_x;
  oy_ = origin_y;
  oz_ = origin_z;
  nx_ = std::max(1, nx);
  ny_ = std::max(1, ny);
  nz_ = std::max(1, nz);
  data_.assign(static_cast<size_t>(nx_) * ny_ * nz_, kUnknown);
}

void VoxelGrid::configureFromBounds(double resolution,
                                    double min_x, double min_y, double min_z,
                                    double max_x, double max_y, double max_z,
                                    double margin_m) {
  const double res = resolution > 0.0 ? resolution : 0.1;
  const double margin = std::max(0.0, margin_m);
  const int nx = static_cast<int>(std::ceil((max_x - min_x + 2.0 * margin) / res)) + 1;
  const int ny = static_cast<int>(std::ceil((max_y - min_y + 2.0 * margin) / res)) + 1;
  const int nz = static_cast<int>(std::ceil((max_z - min_z + 2.0 * margin) / res)) + 1;
  configure(res, min_x - margin, min_y - margin, min_z - margin, nx, ny, nz);
}

void VoxelGrid::clear() { std::fill(data_.begin(), data_.end(), kUnknown); }

bool VoxelGrid::inRange(int ix, int iy, int iz) const {
  return ix >= 0 && iy >= 0 && iz >= 0 && ix < nx_ && iy < ny_ && iz < nz_;
}

uint8_t VoxelGrid::at(int ix, int iy, int iz) const {
  if (!inRange(ix, iy, iz)) return kUnknown;
  return data_[index(ix, iy, iz)];
}

void VoxelGrid::set(int ix, int iy, int iz, uint8_t value) {
  if (!inRange(ix, iy, iz)) return;
  data_[index(ix, iy, iz)] = value;
}

bool VoxelGrid::worldToIndex(double x, double y, double z,
                             int& ix, int& iy, int& iz) const {
  ix = static_cast<int>(std::floor((x - ox_) / res_));
  iy = static_cast<int>(std::floor((y - oy_) / res_));
  iz = static_cast<int>(std::floor((z - oz_) / res_));
  return inRange(ix, iy, iz);
}

void VoxelGrid::indexToWorld(int ix, int iy, int iz,
                             double& x, double& y, double& z) const {
  x = ox_ + (static_cast<double>(ix) + 0.5) * res_;
  y = oy_ + (static_cast<double>(iy) + 0.5) * res_;
  z = oz_ + (static_cast<double>(iz) + 0.5) * res_;
}

void VoxelGrid::insertSurfacePoint(double x, double y, double z,
                                   int clearance_cells) {
  int ix, iy, iz;
  if (!worldToIndex(x, y, z, ix, iy, iz)) return;
  set(ix, iy, iz, kOccupied);
  for (int dz = 1; dz <= clearance_cells; ++dz) {
    if (iz + dz >= nz_) break;
    if (at(ix, iy, iz + dz) == kUnknown) set(ix, iy, iz + dz, kFree);
  }
}

void VoxelGrid::inflate(int xy_radius_cells, int z_radius_cells,
                        bool skip_standing_surfaces) {
  if (xy_radius_cells <= 0 && z_radius_cells <= 0) return;
  const int rxy = std::max(0, xy_radius_cells);
  const int rz = std::max(0, z_radius_cells);
  // Snapshot the occupied set first: inflating while scanning would let newly
  // written INFLATED cells influence later iterations.
  std::vector<int> occupied;
  occupied.reserve(1024);
  for (int iz = 0; iz < nz_; ++iz) {
    for (int iy = 0; iy < ny_; ++iy) {
      const int base = (iz * ny_ + iy) * nx_;
      for (int ix = 0; ix < nx_; ++ix) {
        const int fx = base + ix;
        if (data_[fx] != kOccupied) continue;
        if (skip_standing_surfaces && at(ix, iy, iz + 1) == kFree) continue;
        occupied.push_back(fx);
      }
    }
  }
  if (occupied.empty()) return;
  const int rxy2 = rxy * rxy;
  for (int flat : occupied) {
    const int iz0 = flat / (nx_ * ny_);
    const int rem = flat - iz0 * nx_ * ny_;
    const int iy0 = rem / nx_;
    const int ix0 = rem - iy0 * nx_;
    for (int dz = -rz; dz <= rz; ++dz) {
      const int iz = iz0 + dz;
      if (iz < 0 || iz >= nz_) continue;
      for (int dy = -rxy; dy <= rxy; ++dy) {
        const int iy = iy0 + dy;
        if (iy < 0 || iy >= ny_) continue;
        for (int dx = -rxy; dx <= rxy; ++dx) {
          if (dx * dx + dy * dy > rxy2) continue;
          const int ix = ix0 + dx;
          if (ix < 0 || ix >= nx_) continue;
          const size_t idx = index(ix, iy, iz);
          if (data_[idx] == kFree) data_[idx] = kInflated;
        }
      }
    }
  }
}

void VoxelGrid::deinflate() {
  for (size_t i = 0; i < data_.size(); ++i) {
    if (data_[i] == kInflated) data_[i] = kFree;
  }
}

bool VoxelGrid::hasGroundSupport(int ix, int iy, int iz,
                                 int xy_radius_cells) const {
  const int r = std::max(0, xy_radius_cells);
  const int zl = iz - 1;
  if (zl < 0) return false;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (at(ix + dx, iy + dy, zl) != kOccupied) continue;
      // One occupied cell with known free space directly beneath it may be a
      // thin sheet, and sparse vertical surfaces - a riser's end face, a wall
      // hit every other layer - look exactly like that. Accepting those lets
      // the planner climb straight up a wall.
      //
      // But a floor and a stair tread are one cell thick too (0.10 m slab,
      // 0.189 m riser on a 0.10 m grid), and rejecting them made every flight
      // unplannable. Separate the two by how wide the surface is: a wall hit
      // shows up as one cell per layer, a tread or a slab spans many.
      if (zl - 1 >= 0 && at(ix + dx, iy + dy, zl - 1) == kFree) {
        int span = 0;
        for (int sy = -2; sy <= 2; ++sy)
          for (int sx = -2; sx <= 2; ++sx)
            if (at(ix + dx + sx, iy + dy + sy, zl) == kOccupied) ++span;
        if (span < 8) continue;   // 25-cell box, need a real horizontal face
      }
      return true;
    }
  }
  return false;
}

int VoxelGrid::groundBelow(int ix, int iy, int iz_hint) const {
  if (ix < 0 || iy < 0 || ix >= nx_ || iy >= ny_) return -1;
  int iz = std::min(iz_hint, nz_ - 1);
  // Skip down through unknown/free until the first surface, then step one
  // cell up: that is the cell the robot body would occupy.
  for (; iz >= 0; --iz) {
    if (at(ix, iy, iz) == kOccupied) {
      const int stand = iz + 1;
      return stand < nz_ ? stand : -1;
    }
  }
  return -1;
}

bool VoxelGrid::nearestWalkable(int ix, int iy, int iz, int radius_cells,
                                int& out_x, int& out_y, int& out_z,
                                bool allow_inflated) const {
  const auto usable = [allow_inflated](uint8_t v) {
    return v == kFree || (allow_inflated && v == kInflated);
  };
  if (usable(at(ix, iy, iz))) {
    out_x = ix; out_y = iy; out_z = iz;
    return true;
  }
  const int r = std::max(1, radius_cells);
  int best_d2 = 1 << 30;
  bool found = false;
  for (int dz = -r; dz <= r; ++dz) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int cx = ix + dx, cy = iy + dy, cz = iz + dz;
        if (!usable(at(cx, cy, cz))) continue;
        const int d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best_d2) {
          best_d2 = d2;
          out_x = cx; out_y = cy; out_z = cz;
          found = true;
        }
      }
    }
  }
  return found;
}

}  // namespace octo_planner_ros1
