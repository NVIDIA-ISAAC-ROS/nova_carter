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
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_owls = LaunchConfiguration('launch_owls', default=True)
    launch_owls_arg = DeclareLaunchArgument('launch_owls', default_value=launch_owls,
                                            description='Launch Owl Cameras')
    launch_hawks = LaunchConfiguration('launch_hawks', default=False)
    launch_hawks_arg = DeclareLaunchArgument('launch_hawks', default_value=launch_hawks,
                                             description='Launch Hawk Cameras')
    launch_hesai = LaunchConfiguration('launch_hesai', default=False)
    launch_hesai_arg = DeclareLaunchArgument('launch_hesai', default_value=launch_hesai,
                                             description='Launch Hesai 3D Lidar')
    launch_rplidars = LaunchConfiguration('launch_rplidars', default=False)
    launch_rplidars_arg = DeclareLaunchArgument('launch_rplidars', default_value=launch_rplidars,
                                                description='Launch rplidars')
    launch_segway = LaunchConfiguration('launch_segway', default=True)
    launch_segway_arg = DeclareLaunchArgument('launch_segway', default_value=launch_segway,
                                             description='Launch Segway Driver')
    launch_args = [launch_owls_arg,
                   launch_hawks_arg,
                   launch_hesai_arg,
                   launch_rplidars_arg,
                   launch_segway_arg]

    carter_navigation_launch_include_dir = os.path.join(
        get_package_share_directory('carter_navigation'), 'launch', 'include')

    urdf_static_transform_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/urdf.launch.py'])
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/foxglove_bridge.launch.py'])
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/joystick.launch.py'])
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/twist_mux.launch.py'])
    )

    segway_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/segway.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_segway', 'True')
    )

    carter_container = Node(
        name='carter_container',
        package='rclcpp_components',
        executable='component_container_isolated')

    correlated_timestamp_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/correlated_timestamp_driver.launch.py'])
    )

    owls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/4_owls_resized.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_owls', 'True')
    )

    hawks_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/4_hawks_resized.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_hawks', 'True')
    )

    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/hesai.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_hesai', 'True')
    )

    rplidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/rplidars.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_rplidars', 'True')
    )

    return LaunchDescription(launch_args + [
        urdf_static_transform_publisher_launch,
        foxglove_bridge_launch,
        joystick_launch,
        twist_mux_launch,
        segway_launch,
        carter_container,
        correlated_timestamp_driver_launch,
        owls_launch,
        hawks_launch,
        hesai_launch,
        rplidars_launch
    ])
