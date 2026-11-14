// Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//   http://www.apache.org/licenses/LICENSE-2.0

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_interfaces/ArrivedAtFloor.h>
#include <nav_interfaces/FloorStatus.h>
#include <nav_interfaces/SwitchFloor.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "floor_manager/map_io.h"

#include <XmlRpcValue.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct FloorSpec {
  std::string id;
  std::string map_yaml;
  std::string reloc_bin;
  double elevator_x{0.0};
  double elevator_y{0.0};
  double elevator_yaw{0.0};
  bool has_elevator{false};
  std::vector<std::string> transition_types;
};

double asDouble(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  return 0.0;
}

std::string asString(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
    return static_cast<std::string>(value);
  }
  return "";
}

}  // namespace

class FloorManagerNode {
public:
  FloorManagerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    map_topic_ = pnh_.param<std::string>("map_topic", "/map");
    map_frame_ = pnh_.param<std::string>("map_frame", "map");
    base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
    odin_config_path_ = pnh_.param<std::string>("odin_config_path", "");
    odin_restart_script_ = pnh_.param<std::string>("odin_restart_script", "");
    odin_restart_mode_ = pnh_.param<std::string>("odin_restart_mode", "process");
    odin_restart_unit_ = pnh_.param<std::string>("odin_restart_unit", "odin1-driver");
    sliding_map_clear_service_ =
        pnh_.param<std::string>("sliding_map_clear_service", "/sliding_map_node/clear");
    neupan_stop_topic_ = pnh_.param<std::string>("neupan_stop_topic", "/neupan/stop");
    mission_pause_service_ =
        pnh_.param<std::string>("mission_pause_service", "/waypoint_manager/pause");
    cmd_vel_topic_ = pnh_.param<std::string>("cmd_vel_topic", "/cmd_vel");
    plan_service_ = pnh_.param<std::string>("plan_service", "/map_planner/plan");
    reloc_timeout_sec_ = pnh_.param("reloc_timeout_sec", 120.0);
    reloc_stable_sec_ = pnh_.param("reloc_stable_sec", 2.0);
    planner_ready_timeout_sec_ = pnh_.param("planner_ready_timeout_sec", 8.0);
    inflate_wait_timeout_sec_ = pnh_.param("inflate_wait_timeout_sec", 6.0);
    pause_mission_on_switch_ = pnh_.param("pause_mission_on_switch", true);
    zero_vel_on_switch_ = pnh_.param("zero_vel_on_switch", true);
    initial_floor_ = pnh_.param<std::string>("initial_floor", "");
    auto_publish_initial_map_ = pnh_.param("auto_publish_initial_map", true);
    // ${REPO_ROOT} in floors.yaml is expanded from this parameter, falling back
    // to the GO2_ROOT environment exported by the launch scripts.
    repo_root_ = pnh_.param<std::string>("repo_root", "");
    if (repo_root_.empty()) {
      const char* env_root = std::getenv("GO2_ROOT");
      if (env_root != nullptr) repo_root_ = env_root;
    }

    if (!loadFloors()) {
      ROS_FATAL("[floor_manager] no floors configured; set ~floors (rosparam) or "
                "~map_yaml + ~reloc_bin for a single floor");
      ros::shutdown();
      return;
    }

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
    status_pub_ = pnh_.advertise<nav_interfaces::FloorStatus>("status", 1, true);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1, false);
    stop_pub_ = nh_.advertise<std_msgs::Empty>(neupan_stop_topic_, 1, false);

    inflated_sub_ = nh_.subscribe(
        "/inflated_map", 1, &FloorManagerNode::inflatedMapCb, this);

    switch_srv_ = pnh_.advertiseService(
        "switch_floor", &FloorManagerNode::switchFloorCb, this);
    arrived_srv_ = pnh_.advertiseService(
        "arrived_at_floor", &FloorManagerNode::arrivedAtFloorCb, this);

    zero_vel_timer_ = nh_.createTimer(
        ros::Duration(0.1), &FloorManagerNode::zeroVelTimer, this);
    status_timer_ = nh_.createTimer(
        ros::Duration(1.0), &FloorManagerNode::statusTimer, this);

    if (!initial_floor_.empty() && auto_publish_initial_map_) {
      const FloorSpec* spec = findFloor(initial_floor_);
      if (spec == nullptr) {
        ROS_WARN("[floor_manager] initial_floor '%s' is not in the floor list",
                 initial_floor_.c_str());
      } else {
        std::string error;
        if (publishMapForFloor(*spec, error)) {
          current_floor_ = spec->id;
          detail_ = "initial map loaded";
          ROS_INFO("[floor_manager] initial floor '%s' map published from %s",
                   current_floor_.c_str(), spec->map_yaml.c_str());
        } else {
          ROS_ERROR("[floor_manager] failed to load initial map: %s",
                    error.c_str());
        }
      }
    } else if (!initial_floor_.empty()) {
      current_floor_ = initial_floor_;
    }
    publishStatus();
  }

private:
  bool loadFloors() {
    XmlRpc::XmlRpcValue floors;
    if (pnh_.getParam("floors", floors) &&
        floors.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < floors.size(); ++i) {
        XmlRpc::XmlRpcValue& entry = floors[i];
        if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
        if (!entry.hasMember("id") || !entry.hasMember("map_yaml") ||
            !entry.hasMember("reloc_bin")) {
          ROS_WARN("[floor_manager] floor entry %d is missing id/map_yaml/reloc_bin",
                   i);
          continue;
        }
        FloorSpec spec;
        spec.id = asString(entry["id"]);
        spec.map_yaml = expandPath(asString(entry["map_yaml"]));
        spec.reloc_bin = expandPath(asString(entry["reloc_bin"]));
        // Generic transition points (lift / stairs / ramp). floor_manager does
        // not plan to them, it only needs to know they exist for logging and
        // for refusing a switch to a floor with no way out.
        if (entry.hasMember("transitions") &&
            entry["transitions"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
          XmlRpc::XmlRpcValue& list = entry["transitions"];
          for (int j = 0; j < list.size(); ++j) {
            XmlRpc::XmlRpcValue& raw = list[j];
            if (raw.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
            spec.transition_types.push_back(
                raw.hasMember("type") ? asString(raw["type"]) : "lift");
          }
        }
        if (entry.hasMember("elevator") &&
            entry["elevator"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          XmlRpc::XmlRpcValue& elevator = entry["elevator"];
          spec.has_elevator = true;
          if (elevator.hasMember("x")) spec.elevator_x = asDouble(elevator["x"]);
          if (elevator.hasMember("y")) spec.elevator_y = asDouble(elevator["y"]);
          if (elevator.hasMember("yaw")) {
            spec.elevator_yaw = asDouble(elevator["yaw"]);
          }
        }
        if (spec.transition_types.empty() && spec.has_elevator) {
          spec.transition_types.push_back("lift");
        }
        ROS_INFO("[floor_manager] floor '%s': %zu transition point(s)%s",
                 spec.id.c_str(), spec.transition_types.size(),
                 spec.has_elevator ? " (legacy elevator block)" : "");
        floor_order_.push_back(spec.id);
        floors_[spec.id] = spec;
      }
    }

    // Single-floor fallback keeps the node usable without a floors file.
    if (floors_.empty()) {
      const std::string id = pnh_.param<std::string>("single_floor_id", "default");
      const std::string map_yaml = pnh_.param<std::string>("map_yaml", "");
      const std::string reloc_bin = pnh_.param<std::string>("reloc_bin", "");
      if (map_yaml.empty() || reloc_bin.empty()) return false;
      FloorSpec spec;
      spec.id = id;
      spec.map_yaml = expandPath(map_yaml);
      spec.reloc_bin = expandPath(reloc_bin);
      floors_[id] = spec;
      floor_order_.push_back(id);
    }
    return !floors_.empty();
  }

  const FloorSpec* findFloor(const std::string& id) const {
    auto it = floors_.find(id);
    return it == floors_.end() ? nullptr : &it->second;
  }

  std::string expandPath(const std::string& value) const {
    if (repo_root_.empty()) return value;
    std::string result = value;
    const std::string token = "${REPO_ROOT}";
    std::size_t pos = 0;
    while ((pos = result.find(token, pos)) != std::string::npos) {
      result.replace(pos, token.size(), repo_root_);
      pos += repo_root_.size();
    }
    return result;
  }

  void inflateTimerReset() {
    last_inflated_at_ = ros::Time(0);
  }

  void inflatedMapCb(const nav_msgs::OccupancyGridConstPtr& msg) {
    last_inflated_at_ = ros::Time::now();
    inflated_width_ = msg->info.width;
    inflated_height_ = msg->info.height;
  }

  void publishStatus() {
    nav_interfaces::FloorStatus status;
    status.current_floor = current_floor_;
    status.target_floor = target_floor_;
    status.switching = switching_.load();
    status.detail = detail_;
    status_pub_.publish(status);
  }

  void statusTimer(const ros::TimerEvent&) { publishStatus(); }

  void zeroVelTimer(const ros::TimerEvent&) {
    if (!switching_.load() || !zero_vel_on_switch_) return;
    geometry_msgs::Twist twist;  // zero-initialised
    cmd_vel_pub_.publish(twist);
  }

  void publishZeroVel(int repeats) {
    if (!zero_vel_on_switch_) return;
    geometry_msgs::Twist twist;
    for (int i = 0; i < repeats; ++i) {
      cmd_vel_pub_.publish(twist);
      ros::Duration(0.02).sleep();
    }
  }

  void stopLocalPlanner() {
    std_msgs::Empty empty;
    stop_pub_.publish(empty);
  }

  bool pauseMission(std::string& why) {
    if (!pause_mission_on_switch_) return true;
    std_srvs::Trigger srv;
    if (!ros::service::call(mission_pause_service_, srv)) {
      why = "pause service unavailable: " + mission_pause_service_;
      return false;
    }
    if (!srv.response.success) {
      // Already idle / paused is acceptable for a floor switch.
      why = "pause returned: " + srv.response.message;
      return true;
    }
    return true;
  }

  bool updateOdinConfig(const std::string& reloc_bin, std::string& error) {
    if (odin_config_path_.empty()) {
      error = "~odin_config_path is empty";
      return false;
    }
    std::ifstream input(odin_config_path_);
    if (!input) {
      error = "cannot open Odin driver config: " + odin_config_path_;
      return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string original = buffer.str();
    input.close();

    std::ostringstream log;
    log << "updated relocalization_map_abs_path in " << odin_config_path_;

    std::regex reloc_re("(relocalization_map_abs_path\\s*:\\s*)\"[^\"]*\"");
    std::string updated = std::regex_replace(
        original, reloc_re, "$1\"" + reloc_bin + "\"");
    if (updated == original) {
      // Key absent or written without quotes; append explicitly.
      updated += "\n  relocalization_map_abs_path: \"" + reloc_bin + "\"\n";
      log << " (key appended)";
    }

    std::regex mode_re("(custom_map_mode\\s*:\\s*)[0-2]");
    const std::string with_mode = std::regex_replace(updated, mode_re, "$1 2");
    if (with_mode == updated) {
      updated += "\n  custom_map_mode: 2\n";
      log << " (custom_map_mode appended)";
    } else {
      updated = with_mode;
    }

    // Keep a pristine copy for manual rollback, mirroring ons2_go2.sh.
    std::ifstream backup_probe(odin_config_path_ + ".orig");
    if (!backup_probe.good()) {
      std::ofstream backup(odin_config_path_ + ".orig");
      backup << original;
    }
    backup_probe.close();

    std::ofstream output(odin_config_path_);
    if (!output) {
      error = "cannot write Odin driver config: " + odin_config_path_;
      return false;
    }
    output << updated;
    output.close();
    ROS_INFO("[floor_manager] %s -> %s", log.str().c_str(), reloc_bin.c_str());
    return true;
  }

  bool restartOdinDriver(std::string& error) {
    if (odin_restart_script_.empty()) {
      error = "~odin_restart_script is empty; cannot restart the Odin1 driver";
      return false;
    }
    std::ostringstream command;
    command << "bash " << odin_restart_script_ << " --mode " << odin_restart_mode_
            << " --unit " << odin_restart_unit_;
    ROS_INFO("[floor_manager] restarting Odin1 driver: %s", command.str().c_str());
    const int rc = std::system(command.str().c_str());
    if (rc != 0) {
      std::ostringstream message;
      message << "driver restart script failed with exit code " << rc;
      error = message.str();
      return false;
    }
    return true;
  }

  bool waitForTfStable(std::string& error) {
    const ros::Time start = ros::Time::now();
    ros::Time stable_since;
    while (ros::ok()) {
      if (tf_buffer_.canTransform(map_frame_, base_frame_, ros::Time(0),
                                  ros::Duration(0.1))) {
        if (stable_since.isZero()) {
          stable_since = ros::Time::now();
          ROS_INFO("[floor_manager] map->%s transform acquired, waiting %.1fs for stability",
                   base_frame_.c_str(), reloc_stable_sec_);
        } else if ((ros::Time::now() - stable_since).toSec() >= reloc_stable_sec_) {
          ROS_INFO("[floor_manager] relocalization stable");
          return true;
        }
      } else {
        stable_since = ros::Time(0);
      }
      if ((ros::Time::now() - start).toSec() > reloc_timeout_sec_) {
        std::ostringstream message;
        message << "no stable " << map_frame_ << "->" << base_frame_
                << " transform within " << reloc_timeout_sec_ << "s";
        error = message.str();
        return false;
      }
      ros::Duration(0.05).sleep();
    }
    error = "ROS shutdown while waiting for relocalization";
    return false;
  }

  bool publishMapForFloor(const FloorSpec& spec, std::string& error) {
    nav_msgs::OccupancyGrid grid;
    if (!floor_manager::loadOccupancyGridFromYaml(spec.map_yaml, grid, error)) {
      return false;
    }
    grid.header.frame_id = map_frame_;
    grid.header.stamp = ros::Time::now();
    map_published_at_ = grid.header.stamp;
    map_pub_.publish(grid);
    ROS_INFO("[floor_manager] published /map for floor '%s': %dx%d @ %.3fm",
             spec.id.c_str(), grid.info.width, grid.info.height,
             grid.info.resolution);
    return true;
  }

  bool waitForPlannerInflate(std::string& error) {    const ros::Time start = ros::Time::now();
    while (ros::ok()) {
      if (!last_inflated_at_.isZero() && last_inflated_at_ >= map_published_at_) {
        ROS_INFO("[floor_manager] map_planner re-inflated the new map (%ux%u)",
                 inflated_width_, inflated_height_);
        return true;
      }
      if ((ros::Time::now() - start).toSec() > inflate_wait_timeout_sec_) {
        // Not fatal: map_planner may be mid-plan. Mission stays paused until
        // the caller resumes, so a slightly late inflate is acceptable.
        error = "map_planner did not re-publish /inflated_map in time";
        return false;
      }
      ros::Duration(0.05).sleep();
    }
    error = "ROS shutdown while waiting for /inflated_map";
    return false;
  }

  bool waitForPlanService(std::string& error) {
    const ros::Time start = ros::Time::now();
    while (ros::ok()) {
      if (ros::service::exists(plan_service_, false)) return true;
      if ((ros::Time::now() - start).toSec() > planner_ready_timeout_sec_) {
        error = "global plan service not available: " + plan_service_;
        return false;
      }
      ros::Duration(0.1).sleep();
    }
    error = "ROS shutdown while waiting for the plan service";
    return false;
  }

  void clearSlidingMap() {
    std_srvs::Trigger srv;
    if (ros::service::call(sliding_map_clear_service_, srv)) {
      ROS_INFO("[floor_manager] sliding_map cleared: %s",
               srv.response.message.c_str());
    } else {
      ROS_WARN("[floor_manager] sliding_map clear service unavailable: %s",
               sliding_map_clear_service_.c_str());
    }
  }

  bool switchFloorCb(nav_interfaces::SwitchFloor::Request& req,
                     nav_interfaces::SwitchFloor::Response& res) {
    std::lock_guard<std::mutex> lock(switch_mutex_);
    return doSwitchFloor(req, res);
  }

  // Caller must hold switch_mutex_.
  bool doSwitchFloor(nav_interfaces::SwitchFloor::Request& req,
                     nav_interfaces::SwitchFloor::Response& res) {
    const ros::Time start = ros::Time::now();
    res.success = false;
    res.floor = req.floor;

    const FloorSpec* spec = findFloor(req.floor);
    if (spec == nullptr) {
      res.message = "unknown floor '" + req.floor + "'";
      return true;
    }
    if (req.floor == current_floor_ && !req.keep_mission_paused) {
      res.success = true;
      res.message = "already on floor " + req.floor;
      res.elapsed_sec = 0.0;
      return true;
    }

    switching_.store(true);
    target_floor_ = req.floor;
    detail_ = "stopping motion";
    publishStatus();

    // 1. Stop motion and local planner before the sensor goes away.
    stopLocalPlanner();
    publishZeroVel(10);

    // 2. Pause the mission (best effort: an already paused mission is fine).
    std::string pause_note;
    if (!pauseMission(pause_note)) {
      ROS_WARN("[floor_manager] %s", pause_note.c_str());
    }

    // 3. Point the Odin1 driver at the new .bin.
    detail_ = "swapping relocalization map";
    publishStatus();
    std::string error;
    if (!updateOdinConfig(spec->reloc_bin, error)) {
      res.message = error;
      switching_.store(false);
      detail_ = error;
      publishStatus();
      res.elapsed_sec = (ros::Time::now() - start).toSec();
      return true;
    }

    // 4. Restart the driver so it picks up the new map.
    detail_ = "restarting Odin1 driver";
    publishStatus();
    if (!restartOdinDriver(error)) {
      res.message = error;
      switching_.store(false);
      detail_ = error;
      publishStatus();
      res.elapsed_sec = (ros::Time::now() - start).toSec();
      return true;
    }

    // 5. Wait for relocalization (stable map->base_link).
    detail_ = "waiting for relocalization";
    publishStatus();
    if (!waitForTfStable(error)) {
      res.message = error;
      switching_.store(false);
      detail_ = error;
      publishStatus();
      res.elapsed_sec = (ros::Time::now() - start).toSec();
      return true;
    }

    // 6. Publish the new occupancy grid.
    detail_ = "publishing floor map";
    publishStatus();
    inflateTimerReset();
    if (!publishMapForFloor(*spec, error)) {
      res.message = error;
      switching_.store(false);
      detail_ = error;
      publishStatus();
      res.elapsed_sec = (ros::Time::now() - start).toSec();
      return true;
    }

    if (!waitForPlannerInflate(error)) {
      ROS_WARN("[floor_manager] %s", error.c_str());
    }
    if (!waitForPlanService(error)) {
      ROS_WARN("[floor_manager] %s", error.c_str());
    }

    // 7. Forget obstacles from the previous floor, reset NeuPAN.
    clearSlidingMap();
    stopLocalPlanner();
    publishZeroVel(5);

    current_floor_ = spec->id;
    target_floor_.clear();
    switching_.store(false);
    detail_ = "ready";
    publishStatus();

    res.success = true;
    res.message = "floor switched to " + spec->id;
    res.elapsed_sec = (ros::Time::now() - start).toSec();
    ROS_INFO("[floor_manager] switch to '%s' completed in %.1fs", spec->id.c_str(),
             res.elapsed_sec);
    return true;
  }

  bool arrivedAtFloorCb(nav_interfaces::ArrivedAtFloor::Request& req,
                        nav_interfaces::ArrivedAtFloor::Response& res) {
    // Arrival confirmation is the manual counterpart of switch_floor: the
    // operator (or a lift integration) asserts that the robot is physically on
    // `floor`, so the map/bin swap can start.
    std::lock_guard<std::mutex> lock(switch_mutex_);
    nav_interfaces::SwitchFloor switch_req;
    switch_req.floor = req.floor;
    switch_req.keep_mission_paused = false;
    nav_interfaces::SwitchFloor switch_res;
    doSwitchFloor(switch_req, switch_res);

    res.success = switch_res.success;
    res.message = switch_res.message;
    res.floor = switch_res.floor;
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher map_pub_;
  ros::Publisher status_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher stop_pub_;
  ros::Subscriber inflated_sub_;
  ros::ServiceServer switch_srv_;
  ros::ServiceServer arrived_srv_;
  ros::Timer zero_vel_timer_;
  ros::Timer status_timer_;

  std::map<std::string, FloorSpec> floors_;
  std::vector<std::string> floor_order_;

  std::mutex switch_mutex_;
  std::atomic<bool> switching_{false};
  std::string current_floor_;
  std::string target_floor_;
  std::string detail_{"idle"};

  std::string map_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odin_config_path_;
  std::string odin_restart_script_;
  std::string odin_restart_mode_;
  std::string odin_restart_unit_;
  std::string sliding_map_clear_service_;
  std::string neupan_stop_topic_;
  std::string mission_pause_service_;
  std::string cmd_vel_topic_;
  std::string plan_service_;
  std::string initial_floor_;
  std::string repo_root_;
  bool auto_publish_initial_map_{true};
  bool pause_mission_on_switch_{true};
  bool zero_vel_on_switch_{true};
  double reloc_timeout_sec_{120.0};
  double reloc_stable_sec_{2.0};
  double planner_ready_timeout_sec_{8.0};
  double inflate_wait_timeout_sec_{6.0};

  ros::Time last_inflated_at_;
  ros::Time map_published_at_;
  uint32_t inflated_width_{0};
  uint32_t inflated_height_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "floor_manager");
  FloorManagerNode node;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
