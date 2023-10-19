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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    carter_navigation_maps_dir = os.path.join(
        get_package_share_directory('carter_navigation'), 'maps')
    map = LaunchConfiguration('map')
    map_arg = DeclareLaunchArgument('map', default_value=os.path.join(
        carter_navigation_maps_dir, 'nvidia_galileo_hubble.yaml'))

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[map, {
            'loc_result_frame': 'map',
            'map_yaml_path': map,
        }],
        remappings=[('localization_result', '/initialpose')])

    load_isaac_ros_map_localization = LoadComposableNodes(
        target_container='carter_container',
        composable_node_descriptions=[
            occupancy_grid_localizer_node
        ]
    )

    return LaunchDescription([
        map_arg,
        load_isaac_ros_map_localization
    ])
