# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_segway_rmp LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

execute_process(COMMAND uname -m COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE
)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Eigen
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# segway_rmp_node
ament_auto_add_library(segway_rmp_node SHARED src/segway_rmp_node.cpp)
target_compile_definitions(segway_rmp_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(segway_rmp_node "nvidia::isaac_ros::segway_rmp::SegwayRMPNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::segway_rmp::SegwayRMPNode;$<TARGET_FILE:segway_rmp_node>\n")

# Link to segway driver to call shutdown functions
if(${ARCHITECTURE} STREQUAL "aarch64")
target_link_libraries(segway_rmp_node
  ${PROJECT_SOURCE_DIR}/../RMP220-SDK/LibAPI/Nova_Carter_v2.3/libctrl_arm64-v8a.so
)
endif()  # aarch64

target_include_directories(segway_rmp_node PRIVATE
  ${PROJECT_SOURCE_DIR}/../RMP220-SDK/LibAPI/Nova_Carter_v2.3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  if(${ARCHITECTURE} STREQUAL "aarch64")
    add_launch_test(test/isaac_ros_segway_rmp_pol_test.py)
  endif()

endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
