// Copyright 2026 Manifold Tech Ltd.
//
// ROS1 node that exposes a 3D occupancy planner on the exact same service
// contract as map_planner (`nav_interfaces/PlanPath`), so mission_manager can
// switch between the 2D and the 3D backend by changing one parameter.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "nav_interfaces/PlanPath.h"
#include "octo_planner_ros1/astar3d.h"
#include "octo_planner_ros1/voxel_grid.h"

namespace octo_planner_ros1 {

class OctoPlannerNode {
 public:
  OctoPlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("map_frame", map_frame_, std::string("map"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    pnh_.param<double>("resolution", resolution_, 0.10);
    pnh_.param<std::string>("pcd_file", pcd_file_, std::string(""));
    pnh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/odin1/cloud_slam"));
    pnh_.param<bool>("accumulate_cloud", accumulate_cloud_, true);
    pnh_.param<int>("max_points_per_cloud", max_points_per_cloud_, 20000);
    pnh_.param<int>("clearance_cells", clearance_cells_, 20);
    pnh_.param<double>("robot_radius", robot_radius_, 0.25);
    pnh_.param<int>("inflation_z_cells", inflation_z_cells_, 0);
    pnh_.param<int>("max_iterations", max_iterations_, 500000);
    pnh_.param<bool>("require_ground_support", require_ground_support_, true);
    pnh_.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells_, 0);
    pnh_.param<int>("snap_search_radius_cells", snap_search_radius_cells_, 12);
    pnh_.param<int>("max_step_z_cells", max_step_z_cells_, 3);
    pnh_.param<double>("z_bias", z_bias_, 0.0);
    pnh_.param<double>("grid_margin_m", grid_margin_m_, 1.0);
    pnh_.param<std::string>("service_name", service_name_,
                            std::string("/map_planner/plan"));
    pnh_.param<bool>("publish_voxel_markers", publish_markers_, false);
    pnh_.param<double>("tf_timeout_sec", tf_timeout_sec_, 0.2);

    if (!pcd_file_.empty()) {
      loadPcd(pcd_file_);
    } else {
      cloud_sub_ = nh_.subscribe(cloud_topic_, 5,
                                 &OctoPlannerNode::cloudCallback, this);
    }

    plan_srv_ = nh_.advertiseService(service_name_,
                                     &OctoPlannerNode::planService, this);
    path3d_pub_ = nh_.advertise<nav_msgs::Path>("octo_planner/path3d", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "octo_planner/occupied_voxels", 1, true);

    ROS_INFO("[octo_planner_ros1] service=%s resolution=%.3f robot_radius=%.2f "
             "source=%s require_ground_support=%d",
             service_name_.c_str(), resolution_, robot_radius_,
             pcd_file_.empty() ? cloud_topic_.c_str() : pcd_file_.c_str(),
             static_cast<int>(require_ground_support_));
    if (pcd_file_.empty() && accumulate_cloud_) {
      ROS_INFO("[octo_planner_ros1] accumulating %s until a plan request arrives",
               cloud_topic_.c_str());
    }
  }

 private:
  void loadPcd(const std::string& file) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) != 0 || cloud->empty()) {
      ROS_ERROR("[octo_planner_ros1] failed to load pcd: %s", file.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    buildGridFromCloud(cloud);
    ROS_INFO("[octo_planner_ros1] loaded %zu points from %s",
             cloud->size(), file.c_str());
  }

  void buildGridFromCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    float min_x = 1e9f, min_y = 1e9f, min_z = 1e9f;
    float max_x = -1e9f, max_y = -1e9f, max_z = -1e9f;
    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
      min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
    }
    if (max_x < min_x) return;
    grid_.configureFromBounds(resolution_, min_x, min_y, min_z,
                              max_x, max_y, max_z, grid_margin_m_);
    ROS_INFO("[octo_planner_ros1] grid %dx%dx%d @ %.2fm  bounds=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f]",
             grid_.nx(), grid_.ny(), grid_.nz(), grid_.resolution(),
             min_x, min_y, min_z, max_x, max_y, max_z);
    const size_t step = std::max<size_t>(1, cloud->size() / 200000);
    for (size_t i = 0; i < cloud->points.size(); i += step) {
      const auto& p = cloud->points[i];
      grid_.insertSurfacePoint(p.x, p.y, p.z, clearance_cells_);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!accumulate_cloud_) return;
    sensor_msgs::PointCloud2 transformed;
    if (msg->header.frame_id == map_frame_) {
      transformed = *msg;
    } else {
      try {
        pcl_ros::transformPointCloud(map_frame_, *msg, transformed, tf_buffer_);
      } catch (tf2::TransformException& ex) {
        if (!tf_warned_) {
          ROS_WARN_THROTTLE(5.0, "[octo_planner_ros1] TF %s->%s unavailable (%s); "
                                "inserting points in the sensor frame",
                            map_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
          tf_warned_ = true;
        }
        transformed = *msg;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(transformed, *cloud);
    if (cloud->empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (grid_.empty()) {
      buildGridFromCloud(cloud);
      first_cloud_stamp_ = msg->header.stamp;
      return;
    }
    const size_t step = std::max<size_t>(
        1, cloud->size() / static_cast<size_t>(std::max(1, max_points_per_cloud_)));
    for (size_t i = 0; i < cloud->points.size(); i += step) {
      const auto& p = cloud->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      grid_.insertSurfacePoint(p.x, p.y, p.z, clearance_cells_);
    }
    ++cloud_count_;
    if (publish_markers_ && cloud_count_ % 50 == 0) publishMarkersLocked();
  }

  void refreshInflationLocked() {
    grid_.deinflate();
    const int rxy = static_cast<int>(std::ceil(robot_radius_ / resolution_));
    grid_.inflate(rxy, inflation_z_cells_, true);
  }

  void publishMarkersLocked() {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = "octo_occupied";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = grid_.resolution();
    m.color.r = 0.9f; m.color.g = 0.35f; m.color.b = 0.25f; m.color.a = 0.55f;
    for (int iz = 0; iz < grid_.nz(); iz += 2) {
      for (int iy = 0; iy < grid_.ny(); iy += 2) {
        for (int ix = 0; ix < grid_.nx(); ix += 2) {
          if (grid_.at(ix, iy, iz) != VoxelGrid::kOccupied) continue;
          geometry_msgs::Point p;
          double x, y, z;
          grid_.indexToWorld(ix, iy, iz, x, y, z);
          p.x = x; p.y = y; p.z = z;
          m.points.push_back(p);
        }
      }
    }
    arr.markers.push_back(m);
    marker_pub_.publish(arr);
  }

  bool lookupRobotPose(VoxelKey& key, double& z_world) {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                      ros::Duration(tf_timeout_sec_));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("[octo_planner_ros1] cannot get %s->%s: %s",
               map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;
    double z = tf.transform.translation.z;
    int ix, iy, iz;
    if (!grid_.worldToIndex(x, y, z, ix, iy, iz)) {
      ROS_WARN("[octo_planner_ros1] robot pose (%.2f,%.2f,%.2f) outside grid", x, y, z);
      return false;
    }
    // base_link sits at body height, roughly 0.3 m above the floor. The
    // planner's ground-support check looks one cell below the candidate, so the
    // start must be the standing cell of the column, not the body centre.
    const int stand = grid_.groundBelow(ix, iy, grid_.nz() - 1);
    if (stand >= 0) iz = stand;
    key.x = ix; key.y = iy; key.z = iz;
    z_world = z;
    return true;
  }

  bool resolveGoalVoxel(const geometry_msgs::PoseStamped& goal, VoxelKey& key) {
    int ix, iy, iz;
    if (!grid_.worldToIndex(goal.pose.position.x, goal.pose.position.y,
                            goal.pose.position.z, ix, iy, iz)) {
      // The caller usually leaves z at 0 while the floor sits higher: retry
      // against the grid's top plane before giving up.
      if (!grid_.worldToIndex(goal.pose.position.x, goal.pose.position.y,
                              grid_.originZ() + grid_.nz() * grid_.resolution(),
                              ix, iy, iz)) {
        return false;
      }
    }
    // Snap vertically onto the highest surface in the column, so a 2D goal
    // lands on the floor rather than inside it.
    int stand = grid_.groundBelow(ix, iy, grid_.nz() - 1);
    if (stand >= 0) iz = stand;
    key.x = ix; key.y = iy; key.z = iz;
    return true;
  }

  nav_msgs::Path toRosPath(const std::vector<VoxelKey>& path,
                           const std_msgs::Header& header) {
    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(path.size());
    double last_x = 1e9, last_y = 1e9, last_z = 1e9;
    for (size_t i = 0; i < path.size(); ++i) {
      double x, y, z;
      grid_.indexToWorld(path[i].x, path[i].y, path[i].z, x, y, z);
      const bool keep = (i == 0) || (i + 1 == path.size()) ||
                        (std::hypot(x - last_x, y - last_y) >= 0.30) ||
                        std::fabs(z - last_z) >= 0.15;
      if (!keep) continue;
      geometry_msgs::PoseStamped p;
      p.header = header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.position.z = z;
      p.pose.orientation.w = 1.0;
      poses.push_back(p);
      last_x = x; last_y = y; last_z = z;
    }
    // Yaw from the local tangent; the last pose keeps the previous heading.
    for (size_t i = 0; i < poses.size(); ++i) {
      const size_t j = std::min(i + 1, poses.size() - 1);
      const double dx = poses[j].pose.position.x - poses[i].pose.position.x;
      const double dy = poses[j].pose.position.y - poses[i].pose.position.y;
      double yaw = 0.0;
      if (std::hypot(dx, dy) > 1e-6) yaw = std::atan2(dy, dx);
      else if (i > 0) {
        yaw = tf2::getYaw(poses[i - 1].pose.orientation);
      }
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      poses[i].pose.orientation = tf2::toMsg(q);
    }
    nav_msgs::Path out;
    out.header = header;
    out.poses = poses;
    return out;
  }

  bool planService(nav_interfaces::PlanPath::Request& req,
                   nav_interfaces::PlanPath::Response& res) {
    if (grid_.empty()) {
      res.success = false;
      res.message = "voxel grid is empty (no cloud received yet)";
      ROS_WARN("[octo_planner_ros1] %s", res.message.c_str());
      return true;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    refreshInflationLocked();

    VoxelKey start;
    double start_z = 0.0;
    if (!lookupRobotPose(start, start_z)) {
      res.success = false;
      res.message = "no map->base_link transform";
      return true;
    }
    VoxelKey goal;
    if (!resolveGoalVoxel(req.goal, goal)) {
      res.success = false;
      res.message = "goal outside the voxel grid";
      return true;
    }

    AStar3D::Params p;
    p.max_iterations = max_iterations_;
    p.require_ground_support = require_ground_support_;
    p.ground_support_xy_radius_cells = ground_support_xy_radius_cells_;
    p.snap_search_radius_cells = snap_search_radius_cells_;
    p.max_step_z_cells = max_step_z_cells_;
    p.z_bias = z_bias_;

    const ros::WallTime t0 = ros::WallTime::now();
    AStar3D::Result r = planner_.plan(grid_, start, goal, p);
    const double ms = (ros::WallTime::now() - t0).toSec() * 1000.0;

    std_msgs::Header header;
    header.frame_id = map_frame_;
    header.stamp = ros::Time::now();

    if (!r.ok) {
      res.success = false;
      res.message = r.message;
      ROS_WARN("[octo_planner_ros1] plan failed: %s (expanded=%d, %.1f ms)",
               r.message.c_str(), r.expanded, ms);
      return true;
    }

    nav_msgs::Path path = toRosPath(r.path, header);
    res.success = true;
    res.message = "3d_astar";
    res.path = path;
    ROS_INFO("[octo_planner_ros1] planned %zu voxels -> %zu poses in %.1f ms "
             "(expanded=%d)", r.path.size(), path.poses.size(), ms, r.expanded);

    path3d_pub_.publish(path);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string map_frame_, base_frame_, pcd_file_, cloud_topic_, service_name_;
  double resolution_ = 0.10;
  double robot_radius_ = 0.25;
  double grid_margin_m_ = 1.0;
  double z_bias_ = 0.0;
  double tf_timeout_sec_ = 0.2;
  int clearance_cells_ = 20;
  int inflation_z_cells_ = 2;
  int max_iterations_ = 500000;
  int ground_support_xy_radius_cells_ = 0;
  int snap_search_radius_cells_ = 12;
  int max_step_z_cells_ = 3;
  int max_points_per_cloud_ = 20000;
  bool accumulate_cloud_ = true;
  bool require_ground_support_ = true;
  bool publish_markers_ = false;
  bool tf_warned_ = false;

  std::mutex mutex_;
  VoxelGrid grid_;
  AStar3D planner_;
  int cloud_count_ = 0;
  ros::Time first_cloud_stamp_;

  ros::Subscriber cloud_sub_;
  ros::ServiceServer plan_srv_;
  ros::Publisher path3d_pub_;
  ros::Publisher marker_pub_;
};

}  // namespace octo_planner_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "octo_planner_ros1");
  octo_planner_ros1::OctoPlannerNode node;
  ros::spin();
  return 0;
}
