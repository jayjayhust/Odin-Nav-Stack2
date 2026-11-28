// Copyright 2026 Manifold Tech Ltd.
//
// terrain_recognizer: the terrain-semantics SLOT of ONS2.
//
// The node answers one question for the rest of the stack - "what is the ground
// ahead?" - and owns everything that is not model-specific: cloud subscription,
// TF into the base frame, region cropping, debouncing, and publishing. Which
// model answers is a parameter (`backend`), injected from
// `terrain_recognizer.backend` in robots/<robot>/robot.yaml, so swapping the
// geometric stair detector for a learned model changes configuration, not code
// or consumers.
//
// Several terrains can coexist in one frame (stairs ahead, a ramp to the right,
// a landing in between). One backend describes one region, so the node runs
// *instances*: named (backend, region) pairs declared in
// `terrain_recognizer.instances` (see config/instances.example.yaml). A profile
// without that key runs the single legacy instance and behaves exactly as
// before.
//
// Publishing contract (see msg/TerrainObservation.msg, msg/TerrainScene.msg):
//   /terrain_recognizer/<instance>/status : latched typed verdict of one
//                                instance (single-instance mode keeps the
//                                historical /terrain_recognizer/status name).
//   /terrain_recognizer/scene  : latched TerrainScene, every instance that has
//                                a verdict this frame - this is what a consumer
//                                asking "what is around me" subscribes to.
//   /stair_gait/status         : latched std_msgs/String JSON, kept
//                                byte-shaped like the legacy stair_gait node
//                                because NeuPAN's escape terrain gate parses
//                                it. It is a REDUCER over the instances whose
//                                region overlaps the forward corridor, so a
//                                side-facing instance cannot suppress escape.
//                                Set `publish_legacy=false` when the legacy
//                                node still runs, so exactly one publisher owns
//                                it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <XmlRpcValue.h>

#include "terrain_recognizer/TerrainObservation.h"
#include "terrain_recognizer/TerrainScene.h"
#include "terrain_recognizer/terrain_backend.h"

namespace terrain_recognizer {

namespace {

std::string joinNames(const std::vector<std::string>& names) {
  std::string out;
  for (const auto& n : names) {
    if (!out.empty()) out += ", ";
    out += n;
  }
  return out;
}

double asDouble(const XmlRpc::XmlRpcValue& v, double fallback) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v);
  return fallback;
}

/// Reads `key` from an XmlRpc struct, keeping `fallback` when the key is absent
/// or not numeric.
double optDouble(const XmlRpc::XmlRpcValue& struct_value, const std::string& key,
                 double fallback) {
  if (struct_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) return fallback;
  if (!struct_value.hasMember(key)) return fallback;
  return asDouble(struct_value[key], fallback);
}

}  // namespace

/// Axis-aligned region in the node's base frame. Defaults are "no restriction".
struct Region {
  double x_min = -1.0e3;
  double x_max = 1.0e3;
  double y_min = -1.0e3;
  double y_max = 1.0e3;

  bool restricted() const { return x_min > -1.0e3 || x_max < 1.0e3 || y_min > -1.0e3 ||
                                   y_max < 1.0e3; }
  bool contains(double x, double y) const {
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
  }

  /// True when the region looks straight ahead - what the legacy
  /// /stair_gait/status verdict is allowed to be decided on. A region that only
  /// clips the edge of the corridor (a sideways instance, say) must not be able
  /// to hold the escape, so the test wants real lateral overlap: at least half
  /// of the narrower of the region and the corridor.
  bool coversForwardCorridor(double half_width) const {
    if (x_max <= 0.0) return false;
    if (half_width <= 0.0) return y_min <= 0.0 && y_max >= 0.0;
    const double overlap = std::min(y_max, half_width) - std::max(y_min, -half_width);
    const double width = y_max - y_min;
    return overlap > 0.0 && overlap >= 0.5 * std::min(width, 2.0 * half_width);
  }
};

class TerrainRecognizerNode {
 public:
  TerrainRecognizerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/odin1/cloud_slam"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    pnh_.param<double>("detect_rate_hz", detect_rate_hz_, 10.0);
    pnh_.param<int>("confirm_frames", confirm_frames_, 3);
    pnh_.param<std::string>("status_topic", status_topic_,
                            std::string("terrain_recognizer/status"));
    pnh_.param<std::string>("scene_topic", scene_topic_,
                            std::string("terrain_recognizer/scene"));
    pnh_.param<std::string>("legacy_status_topic", legacy_status_topic_,
                            std::string("stair_gait/status"));
    pnh_.param<bool>("publish_legacy", publish_legacy_, true);
    pnh_.param<double>("legacy_corridor_half_width", legacy_corridor_half_width_, 0.6);
    confirm_frames_ = std::max(1, confirm_frames_);
    legacy_corridor_half_width_ = std::max(0.0, legacy_corridor_half_width_);

    loadInstances();

    cloud_sub_ = nh_.subscribe(cloud_topic_, 3, &TerrainRecognizerNode::cloudCallback, this);
    scene_pub_ = nh_.advertise<TerrainScene>(scene_topic_, 10, /*latch=*/true);
    if (publish_legacy_ && !legacy_status_topic_.empty()) {
      legacy_pub_ = nh_.advertise<std_msgs::String>(legacy_status_topic_, 10, /*latch=*/true);
    }
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, detect_rate_hz_)),
                             &TerrainRecognizerNode::tick, this);

    std::string described;
    for (const auto& inst : instances_) {
      if (!described.empty()) described += " | ";
      described += inst.spec.name.empty() ? inst.spec.backend : inst.spec.name;
      described += ":";
      described += inst.backend->describe();
    }
    ROS_INFO("[terrain_recognizer] instances(%zu) %s cloud=%s base=%s rate=%.1fHz "
             "confirm=%d scene=%s legacy=%s",
             instances_.size(), described.c_str(), cloud_topic_.c_str(),
             base_frame_.c_str(), detect_rate_hz_, confirm_frames_, scene_topic_.c_str(),
             publish_legacy_ ? legacy_status_topic_.c_str() : "off");
  }

 private:
  struct InstanceSpec {
    std::string name;     // empty in single-instance mode
    std::string backend;  // factory key
    Region region;
    bool legacy = true;   // may vote on /stair_gait/status
  };

  struct Instance {
    InstanceSpec spec;
    TerrainBackendPtr backend;
    ros::Publisher status_pub;
    TerrainResult result;
    bool have_result = false;
    std::string debounce_key;
    int debounce_frames = 0;
  };

  /// Builds the instance list. Two forms, and the list wins when both exist:
  ///   backend: stair_gait                      # one instance, legacy topics
  ///   instances: {front: {backend: ..., roi: ...}, left: {...}}
  /// `~backend` is the default for instances that omit their own backend, so a
  /// site file can list regions only.
  void loadInstances() {
    std::string default_backend;
    pnh_.param<std::string>("backend", default_backend, std::string("stair_gait"));
    // The launch file always sets ~backend, so an instances-only profile (`backend:`
    // absent in robot.yaml) arrives here as an empty string. The reference backend
    // is the fallback for the whole node, not just for the single-instance form.
    if (default_backend.empty()) default_backend = std::string("stair_gait");

    XmlRpc::XmlRpcValue declared;
    const bool multi =
        pnh_.getParam("instances", declared) &&
        declared.getType() == XmlRpc::XmlRpcValue::TypeStruct && declared.size() > 0;

    std::vector<InstanceSpec> specs;
    if (multi) {
      for (auto it = declared.begin(); it != declared.end(); ++it) {
        InstanceSpec spec;
        spec.name = it->first;
        const XmlRpc::XmlRpcValue& v = it->second;
        if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (v.hasMember("backend") && v["backend"].getType() ==
                                            XmlRpc::XmlRpcValue::TypeString) {
            spec.backend = static_cast<std::string>(v["backend"]);
          }
          if (v.hasMember("legacy") && v["legacy"].getType() ==
                                           XmlRpc::XmlRpcValue::TypeBoolean) {
            spec.legacy = static_cast<bool>(v["legacy"]);
          }
          if (v.hasMember("roi") &&
              v["roi"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            const XmlRpc::XmlRpcValue& roi = v["roi"];
            spec.region.x_min = optDouble(roi, "x_min", spec.region.x_min);
            spec.region.x_max = optDouble(roi, "x_max", spec.region.x_max);
            spec.region.y_min = optDouble(roi, "y_min", spec.region.y_min);
            spec.region.y_max = optDouble(roi, "y_max", spec.region.y_max);
          }
        }
        specs.push_back(spec);
      }
    } else {
      InstanceSpec spec;
      spec.backend = default_backend;
      specs.push_back(spec);
    }

    const bool name_topics = multi;
    for (auto& spec : specs) {
      // An instance that only lists a region inherits the node-level backend.
      if (spec.backend.empty()) spec.backend = default_backend;
      Instance inst;
      inst.spec = spec;
      inst.backend = createTerrainBackend(spec.backend);
      if (!inst.backend) {
        throw std::runtime_error(
            "unknown terrain backend '" + spec.backend + "'" +
                (spec.name.empty() ? "" : " (instance '" + spec.name + "')") +
                " (available: " + joinNames(availableTerrainBackends()) +
                "); set terrain_recognizer.backend in robots/<robot>/robot.yaml");
      }
      // Backend parameters are node-global (`~`), so every instance of the same
      // backend shares them; the instance region is what differs.
      inst.backend->configure(pnh_);
      const std::string topic = name_topics ? "terrain_recognizer/" + spec.name + "/status"
                                            : status_topic_;
      inst.status_pub = nh_.advertise<TerrainObservation>(topic, 10, /*latch=*/true);
      instances_.push_back(std::move(inst));
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    sensor_msgs::PointCloud2 in_base;
    try {
      pcl_ros::transformPointCloud(base_frame_, *msg, in_base, tf_buffer_);
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "[terrain_recognizer] TF %s -> %s failed: %s",
                        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(in_base, cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = cloud;
    have_cloud_ = true;
    stamp_ = msg->header.stamp;
  }

  static pcl::PointCloud<pcl::PointXYZ> cropToRegion(const pcl::PointCloud<pcl::PointXYZ>& in,
                                                     const Region& region) {
    if (!region.restricted()) return in;
    pcl::PointCloud<pcl::PointXYZ> out;
    out.reserve(in.size());
    for (const auto& p : in.points) {
      if (region.contains(p.x, p.y)) out.push_back(p);
    }
    out.width = static_cast<uint32_t>(out.size());
    out.height = 1;
    out.is_dense = in.is_dense;
    return out;
  }

  void tick(const ros::TimerEvent&) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ros::Time stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_cloud_) return;
      cloud = latest_;
      stamp = stamp_;
    }

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = base_frame_;

    TerrainScene scene;
    scene.header = header;

    for (auto& inst : instances_) {
      const pcl::PointCloud<pcl::PointXYZ> region_cloud =
          cropToRegion(cloud, inst.spec.region);
      inst.backend->observe(region_cloud, header);
      inst.result = inst.backend->result();
      inst.have_result = true;
      if (!inst.result.valid) continue;
      if (!confirmed(inst.debounce_key, inst.debounce_frames,
                     inst.result.type + "/" + inst.result.legacy_state, confirm_frames_)) {
        continue;
      }
      TerrainObservation msg = toMsg(inst, header);
      inst.status_pub.publish(msg);
      scene.observations.push_back(msg);
    }

    // Only publish a scene with content: a latched empty scene would tell a
    // late-joining consumer that the world is flat.
    if (!scene.observations.empty()) scene_pub_.publish(scene);

    if (legacy_pub_) publishLegacy();
  }

  /// Frame counter: a verdict is published after `need` consecutive identical
  /// ones, so a single misread tread does not flip a consumer's state machine.
  /// Once confirmed it is re-published every tick - consumers of the latched
  /// JSON apply their own freshness check.
  static bool confirmed(std::string& key, int& frames, const std::string& candidate,
                        int need) {
    if (candidate != key) {
      key = candidate;
      frames = 1;
    } else {
      ++frames;
    }
    return frames >= need;
  }

  TerrainObservation toMsg(const Instance& inst, const std_msgs::Header& header) const {
    const TerrainResult& result = inst.result;
    TerrainObservation msg;
    msg.header = header;
    msg.type = result.type;
    msg.confidence = result.confidence;
    msg.steps = result.steps;
    msg.rise_m = result.rise_m;
    msg.span_m = result.span_m;
    msg.valid = result.valid;
    msg.backend = inst.backend->name();
    msg.instance = inst.spec.name;
    msg.legacy_state = result.legacy_state;
    msg.geometry.header = msg.header;
    msg.geometry.polygon.points = clipToRegion(result.region, inst.spec.region);
    return msg;
  }

  /// Keeps the reported region inside the instance's own region: a backend may
  /// analyse a wider box than the instance was asked about.
  static std::vector<geometry_msgs::Point32> clipToRegion(
      const std::vector<geometry_msgs::Point32>& polygon, const Region& region) {
    if (!region.restricted()) return polygon;
    std::vector<geometry_msgs::Point32> out = polygon;
    for (auto& p : out) {
      p.x = static_cast<float>(std::min(std::max(static_cast<double>(p.x), region.x_min),
                                       region.x_max));
      p.y = static_cast<float>(std::min(std::max(static_cast<double>(p.y), region.y_min),
                                       region.y_max));
    }
    return out;
  }

  /// Reduces the instances that can see straight ahead into the single verdict
  /// /stair_gait/status has always carried. A staircase outranks a flat reading
  /// (that is the whole point of the gate); ties break on confidence.
  void publishLegacy() {
    const Instance* chosen = nullptr;
    int chosen_rank = -1;
    for (const auto& inst : instances_) {
      if (!inst.spec.legacy || !inst.have_result || !inst.result.valid) continue;
      if (!inst.spec.region.coversForwardCorridor(legacy_corridor_half_width_)) continue;
      const std::string& state = inst.result.legacy_state;
      const int rank = (state == "up" || state == "down") ? 2 : (state == "flat" ? 1 : 0);
      if (chosen == nullptr || rank > chosen_rank ||
          (rank == chosen_rank && inst.result.confidence > chosen->result.confidence)) {
        chosen = &inst;
        chosen_rank = rank;
      }
    }
    if (chosen == nullptr) return;

    if (!confirmed(legacy_debounce_key_, legacy_debounce_frames_,
                   chosen->result.legacy_state, confirm_frames_)) {
      return;
    }

    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "{\"state\":\"%s\",\"steps\":%d,\"rise\":%.3f,\"span\":%.3f,"
                  "\"confidence\":%.2f,\"backend\":\"%s\",\"instance\":\"%s\"}",
                  chosen->result.legacy_state.c_str(), chosen->result.steps,
                  chosen->result.rise_m, chosen->result.span_m,
                  chosen->result.confidence, chosen->backend->name().c_str(),
                  chosen->spec.name.c_str());
    std_msgs::String out;
    out.data = buf;
    legacy_pub_.publish(out);
  }

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string cloud_topic_, base_frame_;
  double detect_rate_hz_ = 10.0;
  int confirm_frames_ = 3;
  std::string status_topic_, scene_topic_, legacy_status_topic_;
  bool publish_legacy_ = true;
  double legacy_corridor_half_width_ = 0.6;

  std::mutex mutex_;
  pcl::PointCloud<pcl::PointXYZ> latest_;
  bool have_cloud_ = false;
  ros::Time stamp_;

  std::string legacy_debounce_key_;
  int legacy_debounce_frames_ = 0;

  std::vector<Instance> instances_;
  ros::Subscriber cloud_sub_;
  ros::Publisher scene_pub_, legacy_pub_;
  ros::Timer timer_;
};

}  // namespace terrain_recognizer

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_recognizer");
  try {
    terrain_recognizer::TerrainRecognizerNode node;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL("[terrain_recognizer] %s", ex.what());
    return 1;
  }
  return 0;
}
