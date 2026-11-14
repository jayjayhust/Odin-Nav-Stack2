cmake_minimum_required(VERSION 3.0.2)
project(stair_gait)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(unitree_sdk2 REQUIRED)
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
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs geometry_msgs
                 pcl_ros pcl_conversions tf2 tf2_ros
)

include_directories(
  include
  /usr/local/include/unitree
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  /usr/local/lib
)

add_executable(stair_gait_node
  src/stair_gait_node.cpp
  src/stair_detector.cpp
)
add_dependencies(stair_gait_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(stair_gait_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  unitree_sdk2
  ddsc
  ddscxx
  Threads::Threads
)
