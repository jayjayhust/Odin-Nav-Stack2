# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0

cmake_minimum_required(VERSION 3.0.2)
project(floor_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_interfaces
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs nav_interfaces
)

add_executable(floor_manager_node
  src/floor_manager_node.cpp
  src/map_io.cpp
)
target_include_directories(floor_manager_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(floor_manager_node ${catkin_LIBRARIES})
add_dependencies(floor_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS floor_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
