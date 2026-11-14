// Copyright 2026 Manifold Tech Ltd.
//
// Stair semantics + gait handover for Go2.
//
// The node answers two questions independently:
//   1. is the ground ahead a staircase, and is it up or down?  (StairDetector)
//   2. may we hand the robot over to the climbing gait?        (explicit gate)
//
// The second question is deliberately hard to answer yes: enable_actuation is
// false by default, the API ids come from parameters, and every service call
// reports the raw return code so a wrong id shows up as an error instead of a
// silent no-op.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unitree/robot/channel/channel_factory.hpp>

#include "stair_gait/ext_sport_client.h"
#include "stair_gait/stair_detector.h"

namespace stair_gait {

class StairGaitNode {
 public:
  StairGaitNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/odin1/cloud_slam"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    pnh_.param<double>("detect_rate_hz", detect_rate_hz_, 10.0);
    pnh_.param<bool>("enable_actuation", enable_actuation_, false);
    pnh_.param<bool>("auto_switch", auto_switch_, false);
    pnh_.param<int>("switch_gait_api_id", switch_gait_api_id_, -1);
    pnh_.param<int>("mode_api_id", mode_api_id_, -1);
    pnh_.param<int>("gait_forward_climb", gait_forward_climb_, 3);
    pnh_.param<int>("gait_reverse_climb", gait_reverse_climb_, 4);
    pnh_.param<int>("gait_trot", gait_trot_, 1);
    pnh_.param<int>("confirm_frames", confirm_frames_, 3);
    pnh_.param<std::string>("network_interface", network_interface_, std::string(""));

    pnh_.param<double>("roi_x_min", roi_.x_min, 0.80);
    pnh_.param<double>("roi_x_max", roi_.x_max, 3.20);
    pnh_.param<double>("roi_y_half", roi_.y_half, 0.60);
    pnh_.param<double>("roi_z_min", roi_.z_min, -0.75);
    pnh_.param<double>("roi_z_max", roi_.z_max, 0.60);
    pnh_.param<double>("bin_m", roi_.bin_m, 0.05);
    pnh_.param<int>("min_bin_points", roi_.min_bin_points, 3);
    pnh_.param<double>("min_rise_m", roi_.min_rise_m, 0.08);
    pnh_.param<double>("max_rise_m", roi_.max_rise_m, 0.30);
    pnh_.param<int>("min_steps", roi_.min_steps, 2);
    pnh_.param<double>("max_rise_spread_m", roi_.max_rise_spread_m, 0.06);

    cloud_sub_ = nh_.subscribe(cloud_topic_, 3, &StairGaitNode::cloudCallback, this);
    status_pub_ = nh_.advertise<std_msgs::String>("stair_gait/status", 10, true);

    enter_up_srv_ = nh_.advertiseService("stair_gait/enter_climb",
                                         &StairGaitNode::enterClimb, this);
    enter_down_srv_ = nh_.advertiseService("stair_gait/enter_descend",
                                           &StairGaitNode::enterDescend, this);
    leave_srv_ = nh_.advertiseService("stair_gait/leave",
                                      &StairGaitNode::leaveStairs, this);
    walkstair_srv_ = nh_.advertiseService("stair_gait/walkstair_mode",
                                          &StairGaitNode::walkStairMode, this);

    if (enable_actuation_) {
      initSdk();
    } else {
      ROS_WARN("[stair_gait] enable_actuation=false: services will refuse to "
               "touch the robot. Set it to true only after confirming the API ids.");
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, detect_rate_hz_)),
                             &StairGaitNode::tick, this);
    ROS_INFO("[stair_gait] cloud=%s base=%s api_ids{gait=%d mode=%d} "
             "actuation=%d auto_switch=%d",
             cloud_topic_.c_str(), base_frame_.c_str(),
             switch_gait_api_id_, mode_api_id_,
             static_cast<int>(enable_actuation_), static_cast<int>(auto_switch_));
  }

 private:
  void initSdk() {
    try {
      if (network_interface_.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0);
      } else {
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
      }
      client_.SetTimeout(10.0f);
      client_.Init();
      sdk_ready_ = true;
      ROS_INFO("[stair_gait] unitree_sdk2 sport client initialised");
    } catch (const std::exception& ex) {
      sdk_ready_ = false;
      ROS_ERROR("[stair_gait] SDK init failed: %s (actuation disabled)", ex.what());
    } catch (...) {
      sdk_ready_ = false;
      ROS_ERROR("[stair_gait] SDK init failed (unknown error)");
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    sensor_msgs::PointCloud2 in_base;
    try {
      pcl_ros::transformPointCloud(base_frame_, *msg, in_base, tf_buffer_);
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "[stair_gait] TF to %s failed: %s",
                        base_frame_.c_str(), ex.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(in_base, cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = cloud;
    have_cloud_ = true;
  }

  void tick(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_cloud_) return;
    StairObservation obs = detector_.detect(latest_, roi_);

    if (obs.valid && obs.state == last_state_) {
      ++same_state_frames_;
    } else if (obs.valid) {
      last_state_ = obs.state;
      same_state_frames_ = 1;
    }
    if (same_state_frames_ < confirm_frames_) return;

    // Publish the stable observation as JSON so mission_manager / Foxglove /
    // a shell hook can consume it without a custom message definition.
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"state\":\"%s\",\"steps\":%d,\"rise\":%.3f,\"span\":%.3f,"
                  "\"confidence\":%.2f,\"api_ids\":{\"gait\":%d,\"mode\":%d},"
                  "\"actuation\":%s}",
                  obs.state.c_str(), obs.steps, obs.rise_m, obs.span_m,
                  obs.confidence, switch_gait_api_id_, mode_api_id_,
                  enable_actuation_ ? "true" : "false");
    std_msgs::String out;
    out.data = buf;
    status_pub_.publish(out);

    if (!auto_switch_ || !enable_actuation_ || !sdk_ready_) return;
    if (!obs.valid) return;

    const int32_t ret = applyGaitForState(obs.state);
    if (ret != 0) {
      ROS_WARN_THROTTLE(2.0, "[stair_gait] auto gait switch for state=%s returned %d",
                        obs.state.c_str(), ret);
    }
  }

  int32_t applyGaitForState(const std::string& state) {
    if (state == "up") return callSwitchGait(gait_forward_climb_);
    if (state == "down") return callSwitchGait(gait_reverse_climb_);
    if (state == "flat") return callSwitchGait(gait_trot_);
    return 0;
  }

  int32_t callSwitchGait(int gait) {
    if (switch_gait_api_id_ < 0) {
      ROS_WARN_THROTTLE(5.0, "[stair_gait] switch_gait_api_id is unset; refusing to call");
      return -1;
    }
    const int32_t ret = client_.SwitchGaitById(switch_gait_api_id_, gait);
    if (ret == 0) ROS_INFO("[stair_gait] switch gait -> %d", gait);
    return ret;
  }

  bool gate(std_srvs::Trigger::Response& res) {
    if (!enable_actuation_) {
      res.success = false;
      res.message = "enable_actuation is false";
      return false;
    }
    if (!sdk_ready_) {
      res.success = false;
      res.message = "unitree_sdk2 client not ready";
      return false;
    }
    if (switch_gait_api_id_ < 0 && mode_api_id_ < 0) {
      res.success = false;
      res.message = "neither switch_gait_api_id nor mode_api_id is set";
      return false;
    }
    return true;
  }

  bool enterClimb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (!gate(res)) return true;
    const int32_t ret = callSwitchGait(gait_forward_climb_);
    res.success = (ret == 0);
    res.message = "switch_gait(" + std::to_string(gait_forward_climb_) +
                  ") -> " + std::to_string(ret);
    return true;
  }

  bool enterDescend(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (!gate(res)) return true;
    const int32_t ret = callSwitchGait(gait_reverse_climb_);
    res.success = (ret == 0);
    res.message = "switch_gait(" + std::to_string(gait_reverse_climb_) +
                  ") -> " + std::to_string(ret);
    return true;
  }

  bool leaveStairs(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (!gate(res)) return true;
    const int32_t ret = callSwitchGait(gait_trot_);
    res.success = (ret == 0);
    res.message = "switch_gait(" + std::to_string(gait_trot_) +
                  ") -> " + std::to_string(ret);
    return true;
  }

  bool walkStairMode(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (!gate(res)) return true;
    if (mode_api_id_ < 0) {
      res.success = false;
      res.message = "mode_api_id is unset";
      return true;
    }
    const int32_t ret = client_.SetModeById(mode_api_id_, "walkstair");
    res.success = (ret == 0);
    res.message = "mode(walkstair) -> " + std::to_string(ret);
    return true;
  }

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string cloud_topic_, base_frame_, network_interface_;
  double detect_rate_hz_ = 10.0;
  bool enable_actuation_ = false;
  bool auto_switch_ = false;
  bool sdk_ready_ = false;
  int switch_gait_api_id_ = -1;
  int mode_api_id_ = -1;
  int gait_forward_climb_ = 3;
  int gait_reverse_climb_ = 4;
  int gait_trot_ = 1;
  int confirm_frames_ = 3;

  std::mutex mutex_;
  pcl::PointCloud<pcl::PointXYZ> latest_;
  bool have_cloud_ = false;
  std::string last_state_ = "unknown";
  int same_state_frames_ = 0;

  StairDetector detector_;
  StairDetectorParams roi_;
  ExtSportClient client_;

  ros::Subscriber cloud_sub_;
  ros::Publisher status_pub_;
  ros::ServiceServer enter_up_srv_, enter_down_srv_, leave_srv_, walkstair_srv_;
  ros::Timer timer_;
};

}  // namespace stair_gait

int main(int argc, char** argv) {
  ros::init(argc, argv, "stair_gait");
  stair_gait::StairGaitNode node;
  ros::spin();
  return 0;
}
