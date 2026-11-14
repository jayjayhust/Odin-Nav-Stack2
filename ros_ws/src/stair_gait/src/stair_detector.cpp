// Copyright 2026 Manifold Tech Ltd.
#include "stair_gait/stair_detector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace stair_gait {
namespace {

// Median of a small window; the raw profile has single-bin dropouts where the
// beam slips between two treads.
double median3(double a, double b, double c) {
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

}  // namespace

StairObservation StairDetector::detect(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                       const StairDetectorParams& p) const {
  StairObservation out;
  if (cloud.empty() || p.bin_m <= 0.0 || p.x_max <= p.x_min) return out;

  const int nbins = static_cast<int>((p.x_max - p.x_min) / p.bin_m) + 1;
  std::vector<double> profile(nbins, std::numeric_limits<double>::quiet_NaN());
  std::vector<int> counts(nbins, 0);
  std::vector<double> sum(nbins, 0.0);

  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    if (pt.x < p.x_min || pt.x > p.x_max) continue;
    if (std::fabs(pt.y) > p.y_half) continue;
    if (pt.z < p.z_min || pt.z > p.z_max) continue;
    const int b = static_cast<int>((pt.x - p.x_min) / p.bin_m);
    if (b < 0 || b >= nbins) continue;
    // The walking surface is the lowest return in the bin: upper returns are
    // walls, door frames or the robot's own legs.
    if (counts[b] == 0 || pt.z < profile[b]) profile[b] = pt.z;
    sum[b] += pt.z;
    ++counts[b];
  }

  // Keep only bins with enough support, then de-spike with a 3-bin median.
  std::vector<double> clean(nbins, std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < nbins; ++i) {
    if (counts[i] >= p.min_bin_points) clean[i] = profile[i];
  }
  std::vector<double> smooth = clean;
  for (int i = 1; i + 1 < nbins; ++i) {
    if (std::isnan(clean[i - 1]) || std::isnan(clean[i]) || std::isnan(clean[i + 1])) continue;
    smooth[i] = median3(clean[i - 1], clean[i], clean[i + 1]);
  }

  // Walk the profile and collect tread edges of the same sign.
  int run_up = 0, run_down = 0;
  int best_up = 0, best_down = 0;
  double span_up_first = 0.0, span_down_first = 0.0;
  double span_up_last = 0.0, span_down_last = 0.0;
  std::vector<double> rises_up, rises_down;
  double prev = std::numeric_limits<double>::quiet_NaN();

  for (int i = 0; i < nbins; ++i) {
    const double z = smooth[i];
    if (std::isnan(z)) continue;
    if (!std::isnan(prev)) {
      const double dz = z - prev;
      const double adz = std::fabs(dz);
      if (adz >= p.min_rise_m && adz <= p.max_rise_m) {
        const double x = p.x_min + i * p.bin_m;
        if (dz > 0) {
          rises_up.push_back(adz);
          if (run_up == 0) span_up_first = x;
          span_up_last = x;
          ++run_up;
          if (run_up > best_up) best_up = run_up;
        } else {
          rises_down.push_back(adz);
          if (run_down == 0) span_down_first = x;
          span_down_last = x;
          ++run_down;
          if (run_down > best_down) best_down = run_down;
        }
      } else if (adz > p.max_rise_m) {
        // A wall or a full storey: reset both runs.
        run_up = run_down = 0;
      }
    }
    prev = z;
  }

  const int chosen = std::max(best_up, best_down);
  if (chosen < p.min_steps) {
    out.state = "flat";
    out.valid = true;
    out.confidence = 1.0;
    return out;
  }

  const bool going_up = best_up >= best_down;
  const std::vector<double>& rises = going_up ? rises_up : rises_down;
  if (rises.empty()) return out;

  double mean = 0.0;
  double lo = 1e9, hi = -1e9;
  for (double r : rises) {
    mean += r;
    lo = std::min(lo, r);
    hi = std::max(hi, r);
  }
  mean /= static_cast<double>(rises.size());
  if (hi - lo > p.max_rise_spread_m) {
    // Treads of very different height: refuse to call it a staircase rather
    // than hand a wrong gait switch to the robot.
    out.state = "unknown";
    out.confidence = 0.2;
    return out;
  }

  out.state = going_up ? "up" : "down";
  out.steps = chosen;
  out.rise_m = mean;
  out.span_m = going_up ? (span_up_last - span_up_first)
                        : (span_down_last - span_down_first);
  out.confidence = std::min(1.0, static_cast<double>(chosen) / 4.0);
  out.valid = true;
  return out;
}

}  // namespace stair_gait
