# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# Set default to Release build unless specified otherwise
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

execute_process(COMMAND uname -m COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE
)
message( STATUS "Architecture: ${ARCHITECTURE}" )

if(NOT ${ARCHITECTURE} STREQUAL "aarch64")
  message( STATUS "Skipping build of package 'isaac_ros_segway_rmp' because not available for architecture: ${ARCHITECTURE}" )
  return()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Eigen and Threads libraries
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

# Include required CMake modules
include(FetchContent)

# Link to segway driver
# Create the lib and include/rmp-sdk directories if they do not exist
set(LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(INCLUDE_RMP_SDK_DIR ${CMAKE_SOURCE_DIR}/include/RMP220-SDK)

if(NOT EXISTS ${LIB_DIR})
  file(MAKE_DIRECTORY ${LIB_DIR})
endif()

if(NOT EXISTS ${INCLUDE_RMP_SDK_DIR})
  file(MAKE_DIRECTORY ${INCLUDE_RMP_SDK_DIR})
endif()

# Disable wonkycase linting for FetchContent functions.
# Note that unindent is required for the pragma to work.
# lint_cmake: -readability/wonkycase

# Download the header and Segway binary from the official GitHub repo
# The below commit hask corresponds to the `v2.0.4` tag
# Commit hask: 61ffa106f27cc17df659653a440989f2e085ea98
# Declare base URL and paths for better readability and fix line too long linting issue
set(BASE_URL "https://github.com/SegwayRoboticsSamples/RMP220-SDK/raw")
set(COMMIT_HASH "61ffa106f27cc17df659653a440989f2e085ea98")
set(LIB_PATH "LibAPI/lib/libctrl_arm64-v8a.so")
set(HEADER_PATH "LibAPI/include/comm_ctrl_navigation.h")

# Full URL construction for the library and header
set(LIB_URL "${BASE_URL}/${COMMIT_HASH}/${LIB_PATH}")
set(HEADER_URL "${BASE_URL}/${COMMIT_HASH}/${HEADER_PATH}")

FetchContent_Declare(
  segway_libctrl
  URL ${LIB_URL}
  DOWNLOAD_NO_EXTRACT TRUE
  DOWNLOAD_DIR ${LIB_DIR}
)

# Download the header and Segway binary from the official GitHub repo
# The below commit hask corresponds to the `v2.0.4` tag
# Commit hask: 61ffa106f27cc17df659653a440989f2e085ea98
FetchContent_Declare(
  segway_header
  URL ${HEADER_URL}
  DOWNLOAD_NO_EXTRACT TRUE
  DOWNLOAD_DIR ${INCLUDE_RMP_SDK_DIR}
)

# Make the content available
FetchContent_MakeAvailable(segway_libctrl segway_header)

target_link_libraries(segway_rmp_node ${CMAKE_SOURCE_DIR}/lib/libctrl_arm64-v8a.so)
install(FILES "${CMAKE_SOURCE_DIR}/lib/libctrl_arm64-v8a.so"
      DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/lib")
set(RMP_SDK_LIB_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/lib")
set_target_properties(segway_rmp_node PROPERTIES
  INSTALL_RPATH "${RMP_SDK_LIB_DIR}"
)

set_target_properties(segway_rmp_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Testing configurations
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


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch config)
