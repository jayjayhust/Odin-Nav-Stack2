# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.0.2)
project(mission_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  atomic_move
  geometry_msgs
  nav_interfaces
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS actionlib atomic_move geometry_msgs nav_interfaces nav_msgs roscpp std_msgs std_srvs tf2_geometry_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_manager_node src/mission_manager_node.cpp)
add_dependencies(mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager_node ${catkin_LIBRARIES})

option(BUILD_REPLAN_GATE_TEST "Build replan install gate unit test" OFF)
if(BUILD_REPLAN_GATE_TEST)
  add_executable(replan_install_gate_test test/replan_install_gate_test.cpp)
  target_include_directories(replan_install_gate_test PRIVATE include)
endif()

# Stuck-escape route-rotation parity gate. Standalone (no ROS, no gtest) like
# the replan gate test, because escape_route_gate.h is deliberately ROS-free.
# It runs the SAME docs/multifloor/sim/escape_contract.fixture.json that the
# simulator's probe_escape_contract.js runs, so the two runtimes are checked
# against one another rather than each asserting its own behaviour.
option(BUILD_ESCAPE_GATE_TEST "Build stuck-escape route-rotation parity test" OFF)
if(BUILD_ESCAPE_GATE_TEST)
  add_executable(test_escape_route_gate test/test_escape_route_gate.cpp)
  target_include_directories(test_escape_route_gate PRIVATE include)
  set(ESCAPE_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/multifloor/sim/escape_contract.fixture.json")
  get_filename_component(ESCAPE_FIXTURE "${ESCAPE_FIXTURE}" ABSOLUTE)
  target_compile_definitions(test_escape_route_gate PRIVATE ESCAPE_FIXTURE_PATH="${ESCAPE_FIXTURE}")
  enable_testing()
  add_test(NAME test_escape_route_gate COMMAND test_escape_route_gate)
endif()
