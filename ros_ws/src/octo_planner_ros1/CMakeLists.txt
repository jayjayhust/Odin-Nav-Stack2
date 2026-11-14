cmake_minimum_required(VERSION 3.0.2)
project(octo_planner_ros1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  nav_interfaces
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(PCL REQUIRED COMPONENTS common io)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES octo_planner_core
  CATKIN_DEPENDS roscpp nav_msgs nav_interfaces sensor_msgs std_msgs
                 geometry_msgs visualization_msgs pcl_ros pcl_conversions
                 tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_library(octo_planner_core
  src/voxel_grid.cpp
  src/astar3d.cpp
)
target_link_libraries(octo_planner_core ${PCL_LIBRARIES})

add_executable(octo_planner_node src/octo_planner_node.cpp)
target_link_libraries(octo_planner_node
  octo_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(octo_planner_node ${catkin_EXPORTED_TARGETS})
