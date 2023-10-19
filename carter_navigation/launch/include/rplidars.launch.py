# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    front_laser_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='front_sllidar_node',
        parameters=[{'channel_type': 'udp',
                     'udp_ip': '192.168.1.2',
                     'udp_port': 8089,
                     'frame_id': 'front_2d_lidar_driver',
                     'inverted': False,
                     'angle_compensate': True,
                     'scan_mode': 'Sensitivity'}],
        remappings=[
            ('scan', 'front/scan'),
        ],
        output='screen',
    )

    back_laser_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='back_sllidar_node',
        parameters=[{'channel_type': 'udp',
                     'udp_ip': '192.168.1.3',
                     'udp_port': 8089,
                     'frame_id': 'back_2d_lidar_driver',
                     'inverted': False,
                     'angle_compensate': True,
                     'scan_mode': 'Sensitivity'}],
        remappings=[
            ('scan', 'back/scan'),
        ],
        output='screen'
    )

    front_laser_driver_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0',
                    '0', '1', '0', 'front_2d_lidar', 'front_2d_lidar_driver'],
    )

    back_laser_driver_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0',
                    '0', '1', '0', 'back_2d_lidar', 'back_2d_lidar_driver'],
    )

    return LaunchDescription([
        front_laser_node,
        back_laser_node,
        front_laser_driver_transform_publisher,
        back_laser_driver_transform_publisher
    ])
