cmake_minimum_required(VERSION 3.0.2)
project(terrain_recognizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  stair_gait
)
find_package(PCL REQUIRED COMPONENTS common)

add_message_files(
  FILES
  TerrainObservation.msg
  TerrainScene.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs message_runtime
                 pcl_ros pcl_conversions tf2 tf2_ros stair_gait
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Backends live in the same executable: each TU registers itself with the
# factory (see include/terrain_recognizer/terrain_backend.h), so adding a new
# model is one source file plus one CMake entry.
add_executable(terrain_recognizer_node
  src/terrain_recognizer_node.cpp
  src/terrain_backend_factory.cpp
  src/backends/stair_gait_backend.cpp
  src/backends/ramp_slope_backend.cpp
  src/backends/dino_v3_backend.cpp
)
add_dependencies(terrain_recognizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(terrain_recognizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  stair_detector
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
