// Copyright 2026 Manifold Tech Ltd.
//
// Second geometric terrain backend: continuous slopes (ramps, dock levellers).
//
// Why this exists next to stair_gait: on an outdoor industrial site the ground
// ahead is usually not a flight of treads but a continuous incline - a loading
// dock leveller, a garage ramp, the apron-to-yard transition, a drainage dip
// between two paved levels. The stair detector deliberately refuses those (no
// tread edges -> "flat"), and "flat" is the one thing a speed or gait planner
// must not see on a 15 % slope.
//
// So the two geometric backends divide the vocabulary instead of overlapping:
//   stair_gait  claims treads      -> stair_up | stair_down | flat | unknown
//   ramp_slope  claims continuous  -> ramp_up  | ramp_down  | flat | unknown
// Both build the same height profile along +x in base_link (lowest return per
// bin = walking surface), but this one fits a straight line through it and
// reports the slope. Treads fail the step and residual gates, so a staircase is
// left to stair_gait. "flat" is simply the slope test at zero, which is why
// this backend is also a usable single-instance recognizer on flat outdoor
// ground.
//
// legacy_state is deliberately "flat" for every ramp verdict: a ramp is drivable
// and safe to back up or turn on, so it must never hold NeuPAN's escape terrain
// gate (that gate exists for stairs).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "terrain_recognizer/terrain_backend.h"

namespace terrain_recognizer {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// Median of three; the raw profile has single-bin dropouts where the beam
/// slips through a gap or between two pavement plates.
double median3(double a, double b, double c) {
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

struct Params {
  // Profile window along +x, lateral corridor and accepted heights, in
  // base_link. The node may already have cropped the cloud to an instance ROI;
  // this window is applied inside that crop (same convention as stair_gait).
  double x_min = 0.60;
  double x_max = 3.00;
  double y_half = 0.50;
  double z_min = -0.80;
  double z_max = 0.80;
  // Profile resolution and support.
  double bin_m = 0.10;
  int min_bin_points = 5;
  // Evidence gates: enough bins, enough x extent, a plane and a drivable pitch.
  int min_bins = 8;
  double min_span_m = 1.00;
  double min_slope_deg = 4.0;
  double max_slope_deg = 22.0;
  double max_rms_m = 0.05;
  /// Largest height jump tolerated between adjacent bins. A continuous slope
  /// cannot jump; a kerb, a dock edge or a tread can. This is what keeps a
  /// single step from being read as a slightly tilted flat floor.
  double max_step_m = 0.06;
};

struct Observation {
  std::string state = "unknown";  // flat | up | down | unknown
  double slope_deg = 0.0;         // + climbs ahead, - drops ahead
  double span_m = 0.0;            // x extent the line was fitted over
  double rise_m = 0.0;            // |slope| * span_m
  double rms_m = 0.0;             // fit residual, the "is it a plane" test
  int bins = 0;
  double confidence = 0.0;
  bool valid = false;
};

/// Height profile -> straight-line fit -> slope verdict. Pure geometry: no ROS,
/// no state, callable from a unit test with a synthetic cloud.
Observation analyse(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Params& p) {
  Observation out;
  if (p.bin_m <= 0.0 || p.x_max <= p.x_min) return out;

  const int nbins = static_cast<int>((p.x_max - p.x_min) / p.bin_m) + 1;
  std::vector<double> z(nbins, kNaN);
  std::vector<double> x(nbins, kNaN);
  std::vector<int> counts(nbins, 0);

  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    if (pt.x < p.x_min || pt.x > p.x_max) continue;
    if (std::fabs(pt.y) > p.y_half) continue;
    if (pt.z < p.z_min || pt.z > p.z_max) continue;
    const int b = static_cast<int>((pt.x - p.x_min) / p.bin_m);
    if (b < 0 || b >= nbins) continue;
    // Lowest return in the bin is the walking surface; upper returns are walls,
    // railings, pallets or the robot's own legs.
    if (counts[b] == 0 || pt.z < z[b]) z[b] = pt.z;
    ++counts[b];
  }

  for (int i = 0; i < nbins; ++i) {
    if (counts[i] < p.min_bin_points) z[i] = kNaN;
    else x[i] = p.x_min + (i + 0.5) * p.bin_m;
  }
  for (int i = 1; i + 1 < nbins; ++i) {
    if (std::isnan(z[i - 1]) || std::isnan(z[i]) || std::isnan(z[i + 1])) continue;
    z[i] = median3(z[i - 1], z[i], z[i + 1]);
  }

  int n = 0;
  double sx = 0.0, sz = 0.0, lo = 1.0e9, hi = -1.0e9;
  for (int i = 0; i < nbins; ++i) {
    if (std::isnan(z[i])) continue;
    ++n;
    sx += x[i];
    sz += z[i];
    lo = std::min(lo, x[i]);
    hi = std::max(hi, x[i]);
  }
  // Too little profile to judge: report "no verdict" so consumers hold their
  // previous state rather than seeing a made-up flat reading.
  if (n < p.min_bins) return out;

  const double mx = sx / n;
  const double mz = sz / n;
  double sxx = 0.0, sxz = 0.0;
  for (int i = 0; i < nbins; ++i) {
    if (std::isnan(z[i])) continue;
    const double dx = x[i] - mx;
    sxx += dx * dx;
    sxz += dx * (z[i] - mz);
  }
  if (!(sxx > 0.0)) return out;
  const double slope = sxz / sxx;

  double sse = 0.0;
  for (int i = 0; i < nbins; ++i) {
    if (std::isnan(z[i])) continue;
    const double e = z[i] - (mz + slope * (x[i] - mx));
    sse += e * e;
  }
  const double rms = std::sqrt(sse / n);
  const double span = hi - lo;

  double max_step = 0.0;
  double prev_z = kNaN;
  for (int i = 0; i < nbins; ++i) {
    if (std::isnan(z[i])) continue;
    if (!std::isnan(prev_z)) max_step = std::max(max_step, std::fabs(z[i] - prev_z));
    prev_z = z[i];
  }

  out.bins = n;
  out.span_m = span;
  out.rms_m = rms;
  out.slope_deg = std::atan(slope) * kRadToDeg;

  if (span < p.min_span_m) return out;  // saw too little of the surface

  if (max_step > p.max_step_m) {
    // A discontinuity, not a slope: a kerb, a dock edge, a tread. Named
    // "unknown" so a speed planner does not treat it as a gentle incline.
    out.state = "unknown";
    out.confidence = 0.2;
    out.valid = true;
    return out;
  }

  if (rms > p.max_rms_m) {
    // Not one plane: treads, a kerb, a dock edge, a pile of material. Refuse to
    // name a slope - stair_gait or a learned backend owns those shapes.
    out.state = "unknown";
    out.confidence = 0.2;
    out.valid = true;
    return out;
  }

  const double adeg = std::fabs(out.slope_deg);
  if (adeg > p.max_slope_deg) {
    // Planar but too steep to drive as a ramp: a wall, a chassis-high step or a
    // stair pitch. Guessing "ramp_up" here would scale speed up a staircase.
    out.state = "unknown";
    out.confidence = 0.2;
    out.valid = true;
    return out;
  }

  const double quality = std::max(0.0, 1.0 - rms / p.max_rms_m);
  if (adeg < p.min_slope_deg) {
    out.state = "flat";
    out.confidence = quality * (1.0 - adeg / p.min_slope_deg);
  } else {
    out.state = out.slope_deg > 0.0 ? "up" : "down";
    out.confidence = quality;
  }
  out.rise_m = std::fabs(slope) * span;
  out.valid = true;
  return out;
}

class RampSlopeBackend : public TerrainBackend {
 public:
  std::string name() const override { return "ramp_slope"; }

  void configure(ros::NodeHandle& pnh) override {
    // Prefixed keys: every instance shares the node's private namespace with the
    // other backends, so bare names such as `x_min` would collide.
    pnh.param<double>("ramp_slope/x_min", params_.x_min, params_.x_min);
    pnh.param<double>("ramp_slope/x_max", params_.x_max, params_.x_max);
    pnh.param<double>("ramp_slope/y_half", params_.y_half, params_.y_half);
    pnh.param<double>("ramp_slope/z_min", params_.z_min, params_.z_min);
    pnh.param<double>("ramp_slope/z_max", params_.z_max, params_.z_max);
    pnh.param<double>("ramp_slope/bin_m", params_.bin_m, params_.bin_m);
    pnh.param<int>("ramp_slope/min_bin_points", params_.min_bin_points,
                   params_.min_bin_points);
    pnh.param<int>("ramp_slope/min_bins", params_.min_bins, params_.min_bins);
    pnh.param<double>("ramp_slope/min_span_m", params_.min_span_m, params_.min_span_m);
    pnh.param<double>("ramp_slope/min_slope_deg", params_.min_slope_deg,
                      params_.min_slope_deg);
    pnh.param<double>("ramp_slope/max_slope_deg", params_.max_slope_deg,
                      params_.max_slope_deg);
    pnh.param<double>("ramp_slope/max_rms_m", params_.max_rms_m, params_.max_rms_m);
    pnh.param<double>("ramp_slope/max_step_m", params_.max_step_m, params_.max_step_m);
    ROS_INFO("[terrain_recognizer] ramp_slope x=[%.2f,%.2f] y=+/-%.2f slope=[%.1f,%.1f]deg "
             "rms<=%.3fm span>=%.2fm",
             params_.x_min, params_.x_max, params_.y_half, params_.min_slope_deg,
             params_.max_slope_deg, params_.max_rms_m, params_.min_span_m);
  }

  std::string describe() const override {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "ramp_slope x=[%.2f,%.2f] y=+/-%.2f slope=[%.1f,%.1f]deg rms<=%.3fm",
                  params_.x_min, params_.x_max, params_.y_half, params_.min_slope_deg,
                  params_.max_slope_deg, params_.max_rms_m);
    return buf;
  }

  void observe(const pcl::PointCloud<pcl::PointXYZ>& cloud_base,
               const std_msgs::Header& header) override {
    (void)header;
    observation_ = analyse(cloud_base, params_);
  }

  TerrainResult result() const override {
    TerrainResult out;
    if (observation_.state == "up") {
      out.type = "ramp_up";
    } else if (observation_.state == "down") {
      out.type = "ramp_down";
    } else {
      out.type = observation_.state;  // flat | unknown
    }
    out.confidence = observation_.confidence;
    out.span_m = observation_.span_m;
    out.rise_m = observation_.rise_m;
    out.steps = 0;
    out.valid = observation_.valid;
    // A ramp is drivable, reversible ground: never hold the escape gate on it.
    out.legacy_state =
        (observation_.state == "up" || observation_.state == "down") ? "flat"
                                                                   : observation_.state;
    out.region = roiPolygon();
    return out;
  }

 private:
  /// Region the line was fitted over, as the polygon consumers can draw. Same
  /// box convention as stair_gait: "where the verdict was measured".
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

  Params params_;
  Observation observation_;
};

TerrainBackendPtr makeRampSlopeBackend() { return TerrainBackendPtr(new RampSlopeBackend()); }

struct Registrar {
  Registrar() { registerTerrainBackend("ramp_slope", &makeRampSlopeBackend); }
} g_registrar;

}  // namespace
}  // namespace terrain_recognizer
