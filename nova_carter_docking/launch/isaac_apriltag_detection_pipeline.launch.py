#!/usr/bin/env python3

# Copyright (c) 2024 Open Navigation LLC
# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_directory('isaac_ros_apriltag'),
        'rviz', 'default.rviz')

    launch_rviz = LaunchConfiguration('launch_rviz')
    image = LaunchConfiguration('image')
    camera_info = LaunchConfiguration('camera_info')

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[('image', image),
                    ('camera_info', camera_info)],
        parameters=[{
            'size': 0.1524,  # 6 inches
            'max_tags': 4,
            'tile_size': 4,
        }],
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
        ],
        output='screen'
    )

    dock_pose_publisher = Node(
        package='nova_carter_docking',
        executable='dock_pose_publisher',
        name='dock_pose_publisher',
        parameters=[{'use_first_detection': True, 'dock_tag_id': 586}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(launch_rviz))

    return launch.LaunchDescription([apriltag_container, dock_pose_publisher, rviz_node])
