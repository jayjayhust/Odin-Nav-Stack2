// Copyright 2026 Manifold Tech Ltd.
//
// Prototype terrain backend: learned semantics (DINOv3-style vision encoder).
//
// STATUS: interface placeholder, roadmap phase 5. The slot exists so a learned
// model can be developed without touching the node, the message or any
// consumer; this translation unit deliberately returns no verdict instead of
// pretending to classify.
//
// What a real implementation must do (and why the parameters are already here):
//   - subscribe to the Odin1 camera (image_topic) and, optionally, the SLAM cloud
//     to lift the 2D prediction into the base frame;
//   - run the encoder at detect_rate_hz or slower, and publish class
//     probabilities as confidence over {flat, stair_up, stair_down, ramp_up,
//     ramp_down, landing}; unknown stays the fallback class;
//   - keep the legacy_state mapping (stair_up -> "up", stair_down -> "down")
//     so NeuPAN's escape terrain gate keeps working;
//   - keep latency inside one detect period: cache the last verdict and return
//     valid=false while a new frame is still being processed.
#include <string>

#include <ros/ros.h>

#include "terrain_recognizer/terrain_backend.h"

namespace terrain_recognizer {
namespace {

class DinoV3Backend : public TerrainBackend {
 public:
  std::string name() const override { return "dino_v3"; }

  void configure(ros::NodeHandle& pnh) override {
    pnh.param<std::string>("model_path", model_path_, std::string(""));
    pnh.param<std::string>("image_topic", image_topic_, std::string("/odin1/image"));
    pnh.param<std::string>("device", device_, std::string("cpu"));
    pnh.param<int>("input_size", input_size_, 518);
    ROS_WARN("[terrain_recognizer] backend=dino_v3 is a prototype stub: no "
             "verdict will be published (model_path='%s' image='%s' device='%s' "
             "input=%d). Implement the encoder in "
             "src/backends/dino_v3_backend.cpp to enable it.",
             model_path_.c_str(), image_topic_.c_str(), device_.c_str(), input_size_);
  }

  std::string describe() const override {
    return "dino_v3 (stub, model_path='" + model_path_ + "' image='" +
           image_topic_ + "' device='" + device_ + "')";
  }

  TerrainResult result() const override {
    // Never invent a class: consumers keep their previous verdict while the
    // backend reports invalid.
    TerrainResult out;
    out.type = "unknown";
    out.legacy_state = "unknown";
    out.valid = false;
    return out;
  }

 private:
  std::string model_path_;
  std::string image_topic_;
  std::string device_;
  int input_size_ = 518;
};

TerrainBackendPtr makeDinoV3Backend() { return TerrainBackendPtr(new DinoV3Backend()); }

struct Registrar {
  Registrar() { registerTerrainBackend("dino_v3", &makeDinoV3Backend); }
} g_registrar;

}  // namespace
}  // namespace terrain_recognizer
