// Copyright 2026 Manifold Tech Ltd.
//
// Backend contract of the terrain-recognizer SLOT.
//
// The node owns the ROS I/O (cloud subscription, TF into base_frame, publishing,
// debouncing); a backend only turns "latest cloud in base_frame" into one
// TerrainResult. That keeps backends ROS-light, makes the geometric detector
// and a learned model interchangeable, and lets every backend be unit-tested
// off-robot by calling observe()/result() directly.
//
// Adding a backend:
//   1. implement TerrainBackend in src/backends/<name>_backend.cpp;
//   2. register it with registerTerrainBackend("<name>", factory) - a file-local
//      static registrar is enough (see the built-ins);
//   3. add the source file to CMakeLists.txt and, if it needs extra libraries,
//      link them only into terrain_recognizer_node.
// Full walkthrough (contract table, skeleton, build/config wiring, offline
// tests): docs/usage/04_custom_terrain_backend.md.
// The selected backend is the `backend` ROS parameter, injected from
// `terrain_recognizer.backend` in robots/<robot>/robot.yaml. When several
// terrains coexist, each entry of `terrain_recognizer.instances` names its own
// backend and region; the node creates one backend object per instance and
// calls observe() on the cloud cropped to that instance's region.
#ifndef TERRAIN_RECOGNIZER_TERRAIN_BACKEND_H
#define TERRAIN_RECOGNIZER_TERRAIN_BACKEND_H

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace terrain_recognizer {

/// One terrain verdict.
///
/// `type` is the label consumers should key on. `legacy_state` carries the
/// flat|up|down|unknown vocabulary of /stair_gait/status so the NeuPAN escape
/// terrain gate keeps working without knowing the richer classes.
struct TerrainResult {
  std::string type = "unknown";
  double confidence = 0.0;
  int steps = 0;
  double rise_m = 0.0;
  double span_m = 0.0;
  bool valid = false;
  std::string legacy_state = "unknown";
  /// Recognized region as a polygon in the node's base frame. Empty when the
  /// backend cannot localize the region.
  std::vector<geometry_msgs::Point32> region;
};

class TerrainBackend {
 public:
  virtual ~TerrainBackend() = default;

  /// Factory key, e.g. "stair_gait".
  virtual std::string name() const = 0;

  /// Read backend-specific parameters from the node's private namespace.
  /// Backends that need extra inputs (camera images, a GPU model) may subscribe
  /// or load here.
  virtual void configure(ros::NodeHandle& pnh) { (void)pnh; }

  /// One-line description for the startup log.
  virtual std::string describe() const { return name(); }

  /// Called at detect_rate_hz with the newest cloud already transformed into
  /// the node's base frame. Must not block longer than one period.
  virtual void observe(const pcl::PointCloud<pcl::PointXYZ>& cloud_base,
                       const std_msgs::Header& header) {
    (void)cloud_base;
    (void)header;
  }

  /// Latest verdict. Called right after observe() on the same thread.
  virtual TerrainResult result() const = 0;
};

using TerrainBackendPtr = std::unique_ptr<TerrainBackend>;
using TerrainBackendFactory = TerrainBackendPtr (*)();

/// Registers (or replaces) a backend under `name`.
void registerTerrainBackend(const std::string& name, TerrainBackendFactory factory);

/// nullptr when `name` is unknown - callers should log availableTerrainBackends().
TerrainBackendPtr createTerrainBackend(const std::string& name);

/// Sorted names, for error messages and `--list` style diagnostics.
std::vector<std::string> availableTerrainBackends();

}  // namespace terrain_recognizer

#endif  // TERRAIN_RECOGNIZER_TERRAIN_BACKEND_H
