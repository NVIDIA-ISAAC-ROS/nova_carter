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

# flake8: noqa: F403,F405
import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import *

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('enable_wheel_odometry')

    actions = args.get_launch_actions()
    actions.append(lu.log_info(['Enabling wheel odometry: ', args.enable_wheel_odometry]))

    segway = ComposableNode(
        package='isaac_ros_segway_rmp',
        plugin='nvidia::isaac_ros::segway_rmp::SegwayRMPNode',
        name='segway_rmp',
        namespace='chassis',
        remappings=[
            ('cmd_vel', '/twist_mux/cmd_vel'),
        ],
        parameters=[{
            'enable_odometry_tf': args.enable_wheel_odometry,
            'enable_statistics': True,
            'topics_list': ['odom'],
            'expected_fps_list': [40.0],
            'jitter_tolerance_us': 50000
        }],
    )

    actions.append(
        ComposableNodeContainer(
            name='segway_rmp_container',
            package='rclcpp_components',
            executable='component_container_mt',
            namespace='',
            prefix='nice -n 1',
            composable_node_descriptions=[segway],
        ))

    return LaunchDescription(actions)
