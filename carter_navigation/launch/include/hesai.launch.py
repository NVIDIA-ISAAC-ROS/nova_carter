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

import math

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    hesai_node = ComposableNode(
        package='isaac_ros_hesai',
        plugin='nvidia::isaac_ros::hesai::HesaiNode',
        name='hesai')

    pointcloud_to_flatscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode',
        name='pointcloud_to_flatscan',
        parameters=[{'min_z': -0.2}])

    flatscan_to_laserscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode',
        name='flatscan_to_laserscan',
        parameters=[{
            'angle_min': -math.pi,
            'angle_max': math.pi,
            'angle_increment': math.pi/720
        }]
    )

    load_hesai_pipeline = LoadComposableNodes(
        target_container='carter_container',
        composable_node_descriptions=[
            hesai_node,
            pointcloud_to_flatscan_node,
            flatscan_to_laserscan_node
        ]
    )

    return LaunchDescription([load_hesai_pipeline])
