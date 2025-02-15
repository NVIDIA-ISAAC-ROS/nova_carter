# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    segway_rmp_node = ComposableNode(
        package='isaac_ros_segway_rmp',
        plugin='nvidia::isaac_ros::segway_rmp::SegwayRMPNode',
        name='segway_rmp',
        parameters=[{
                'enable_diagnostics': True,
                'topics_list': ['odom'],
                'expected_fps_list': [40.0],
                'jitter_tolerance_us': 200000
            }])

    segway_rmp_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='segway_rmp_container',
        namespace='',
        prefix='nice -n 1',
        executable='component_container_mt',
        composable_node_descriptions=[
            segway_rmp_node,
        ],
        output='screen'
    )

    return launch.LaunchDescription([segway_rmp_container])
