// Copyright 2026 Manifold Tech Ltd.
//
// Stair semantics from a single depth/point cloud slice.
//
// The detector does not try to segment a whole staircase. It answers one
// question the gait layer actually needs: "does the ground ahead of the robot
// step up, step down, or stay flat, and by how much per tread?" The answer is
// built from a height profile along +x in base_link, so it is independent of
// SLAM, of the 2D costmap and of the floor the robot currently stands on.
#ifndef STAIR_GAIT_STAIR_DETECTOR_H
#define STAIR_GAIT_STAIR_DETECTOR_H

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace stair_gait {

struct StairObservation {
  std::string state = "unknown";  // flat | up | down | unknown
  int steps = 0;
  double rise_m = 0.0;     // mean tread height from the profile
  double span_m = 0.0;     // x extent covered by the steps
  double confidence = 0.0; // 0..1, share of bins that voted for the sign
  bool valid = false;
};

struct StairDetectorParams {
  double x_min = 0.80;
  double x_max = 3.20;
  double y_half = 0.60;
  double z_min = -0.75;
  double z_max = 0.60;
  double bin_m = 0.05;
  int min_bin_points = 3;
  double min_rise_m = 0.08;
  double max_rise_m = 0.30;
  int min_steps = 2;
  double max_rise_spread_m = 0.06;
};

class StairDetector {
 public:
  StairObservation detect(const pcl::PointCloud<pcl::PointXYZ>& cloud_base,
                          const StairDetectorParams& params) const;
};

}  // namespace stair_gait

#endif  // STAIR_GAIT_STAIR_DETECTOR_H
