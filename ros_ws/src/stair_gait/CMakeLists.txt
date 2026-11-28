cmake_minimum_required(VERSION 3.0.2)
project(stair_gait)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
# unitree_sdk2 is only needed by the gait-handover node. The detector library is
# ROS-free and SDK-free so terrain_recognizer (and any other consumer) can link
# the geometry without the quadruped SDK installed.
find_package(unitree_sdk2 QUIET)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)
find_package(PCL REQUIRED COMPONENTS common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stair_detector
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs geometry_msgs
                 pcl_ros pcl_conversions tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Detector library (libstair_detector.so): no ROS runtime, no unitree_sdk2.
add_library(stair_detector src/stair_detector.cpp)
add_dependencies(stair_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(stair_detector ${PCL_LIBRARIES})

if(unitree_sdk2_FOUND)
  include_directories(/usr/local/include/unitree)
  link_directories(
    ${catkin_LIBRARY_DIRS}
    /usr/local/lib
  )

  add_executable(stair_gait_node src/stair_gait_node.cpp)
  add_dependencies(stair_gait_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(stair_gait_node
    stair_detector
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    unitree_sdk2
    ddsc
    ddscxx
    Threads::Threads
  )
else()
  message(WARNING
    "unitree_sdk2 not found: building libstair_detector only, "
    "stair_gait_node (Go2 gait handover) is skipped.")
endif()
