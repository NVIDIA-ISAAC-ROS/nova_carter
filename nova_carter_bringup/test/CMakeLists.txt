# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

find_package(launch_testing_ament_cmake REQUIRED)

find_package(isaac_ros_test_cmake)

add_launch_test(
    nova_carter_perceptor_single_cam_pol.py
    TARGET nova_carter_perceptor_single_cam_pol
    TIMEOUT 120
)
set_tests_properties(nova_carter_perceptor_single_cam_pol PROPERTIES RESOURCE_LOCK "nova_carter_pol")

add_launch_test(
    nova_carter_perceptor_multi_cam_pol.py
    TARGET nova_carter_perceptor_multi_cam_pol
    TIMEOUT 120
)
set_tests_properties(nova_carter_perceptor_multi_cam_pol PROPERTIES RESOURCE_LOCK "nova_carter_pol")


# Get the path to a dummy bag for use in the tests.
get_dummy_bag_path(DUMMY_BAG_PATH)

add_graph_startup_test(multi_cam_perceptor_dry_run
    launch/perceptor.launch.py
    TIMEOUT 60
    ARGS "mode:=rosbag rosbag:=${DUMMY_BAG_PATH} run_rviz:=False run_foxglove:=False"
)

add_graph_startup_test(single_cam_perceptor_dry_run
    launch/perceptor.launch.py
    TIMEOUT 60
    ARGS "mode:=rosbag rosbag:=${DUMMY_BAG_PATH} run_rviz:=False run_foxglove:=False stereo_camera_configuration:=front_configuration"
)

#NOTE(alexmillane): In this test the graph fails on shutdown terminates with a non-zero exit code
# which causes the test to fail (even though the graph *runs* correctly). This issue is under
# investigation here: https://nvbugspro.nvidia.com/bug/4751762.
# Disabling the test for now. Uncomment when shutdown issue resolved.
# add_graph_startup_test(multi_cam_navigation_dry_run
#     launch/navigation.launch.py
#     TIMEOUT 30
#     ARGS "mode:=simulation run_rviz:=False run_foxglove:=False navigation:=False"
# )

# Add installation of ESS models as a test fixture.
get_isaac_ros_asset_install_script_path(install_ess_models_script install_ess_models isaac_ros_ess_models_install)
ament_add_test(setup_ess_models
    COMMAND "${install_ess_models_script}"
    ENV "ISAAC_ROS_SKIP_ASSET_INSTALL="
    TIMEOUT 1200
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
)

# Register the setup ess models script as a pre-test requirement.
set_tests_properties(setup_ess_models PROPERTIES FIXTURES_SETUP pre_test)
set_tests_properties(nova_carter_perceptor_single_cam_pol PROPERTIES FIXTURES_REQUIRED pre_test)
set_tests_properties(nova_carter_perceptor_multi_cam_pol PROPERTIES FIXTURES_REQUIRED pre_test)
set_tests_properties(multi_cam_perceptor_dry_run PROPERTIES FIXTURES_REQUIRED pre_test)
set_tests_properties(single_cam_perceptor_dry_run PROPERTIES FIXTURES_REQUIRED pre_test)
