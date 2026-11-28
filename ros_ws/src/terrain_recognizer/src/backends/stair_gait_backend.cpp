// Copyright 2026 Manifold Tech Ltd.
//
// Reference terrain backend: the geometric stair detector of the stair_gait
// package, unchanged, wrapped in the slot contract.
//
// This backend is pure geometry on a height profile along +x in base_link, so
// it needs no camera, no GPU and no training data - it is the default and the
// fallback when a learned backend is unavailable. It reports only staircase
// classes today (flat / stair_up / stair_down / unknown): continuous slopes are
// ramp_slope's job (a plane fit on the same profile), and `landing` stays
// reserved for a learned backend that can separate it from flat ground.
#include <cstdio>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "stair_gait/stair_detector.h"
#include "terrain_recognizer/terrain_backend.h"

namespace terrain_recognizer {
namespace {

/// Maps the flat|up|down|unknown vocabulary of stair_gait to the slot's labels.
std::string toType(const std::string& state) {
  if (state == "up") return "stair_up";
  if (state == "down") return "stair_down";
  if (state == "flat") return "flat";
  return "unknown";
}

class StairGaitBackend : public TerrainBackend {
 public:
  std::string name() const override { return "stair_gait"; }

  void configure(ros::NodeHandle& pnh) override {
    pnh.param<double>("roi_x_min", params_.x_min, params_.x_min);
    pnh.param<double>("roi_x_max", params_.x_max, params_.x_max);
    pnh.param<double>("roi_y_half", params_.y_half, params_.y_half);
    pnh.param<double>("roi_z_min", params_.z_min, params_.z_min);
    pnh.param<double>("roi_z_max", params_.z_max, params_.z_max);
    pnh.param<double>("bin_m", params_.bin_m, params_.bin_m);
    pnh.param<int>("min_bin_points", params_.min_bin_points, params_.min_bin_points);
    pnh.param<double>("min_rise_m", params_.min_rise_m, params_.min_rise_m);
    pnh.param<double>("max_rise_m", params_.max_rise_m, params_.max_rise_m);
    pnh.param<int>("min_steps", params_.min_steps, params_.min_steps);
    pnh.param<double>("max_rise_spread_m", params_.max_rise_spread_m,
                      params_.max_rise_spread_m);
  }

  std::string describe() const override {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "stair_gait roi x=[%.2f,%.2f] y=+/-%.2f z=[%.2f,%.2f] "
                  "bin=%.2fm min_steps=%d",
                  params_.x_min, params_.x_max, params_.y_half, params_.z_min,
                  params_.z_max, params_.bin_m, params_.min_steps);
    return buf;
  }

  void observe(const pcl::PointCloud<pcl::PointXYZ>& cloud_base,
               const std_msgs::Header& header) override {
    (void)header;
    observation_ = detector_.detect(cloud_base, params_);
  }

  TerrainResult result() const override {
    TerrainResult out;
    out.type = toType(observation_.state);
    out.confidence = observation_.confidence;
    out.steps = observation_.steps;
    out.rise_m = observation_.rise_m;
    out.span_m = observation_.span_m;
    out.valid = observation_.valid;
    out.legacy_state = observation_.state;
    out.region = roiPolygon();
    return out;
  }

 private:
  /// The analyzer's region of interest, as the polygon consumers can draw.
  /// The detector votes over this whole box, it does not segment the steps, so
  /// this is "where the verdict was measured", not a step contour.
  std::vector<geometry_msgs::Point32> roiPolygon() const {
    auto pt = [](double x, double y) {
      geometry_msgs::Point32 p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      p.z = 0.0f;
      return p;
    };
    return {pt(params_.x_min, -params_.y_half), pt(params_.x_max, -params_.y_half),
            pt(params_.x_max, params_.y_half), pt(params_.x_min, params_.y_half)};
  }

  stair_gait::StairDetector detector_;
  stair_gait::StairDetectorParams params_;
  stair_gait::StairObservation observation_;
};

TerrainBackendPtr makeStairGaitBackend() {
  return TerrainBackendPtr(new StairGaitBackend());
}

struct Registrar {
  Registrar() { registerTerrainBackend("stair_gait", &makeStairGaitBackend); }
} g_registrar;

}  // namespace
}  // namespace terrain_recognizer
