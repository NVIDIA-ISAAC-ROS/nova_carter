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
    carter_navigation_launch_include_dir = os.path.join(
        get_package_share_directory('carter_navigation'), 'launch', 'include')
    carter_navigation_params_dir = os.path.join(
        get_package_share_directory('carter_navigation'), 'params')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')
    mission_client_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_vda5050_nav2_client_bringup'), 'launch')

    launch_owls = LaunchConfiguration('launch_owls', default=False)
    launch_owls_arg = DeclareLaunchArgument('launch_owls', default_value=launch_owls,
                                            description='Launch Owl Cameras')
    launch_hawks = LaunchConfiguration('launch_hawks', default=False)
    launch_hawks_arg = DeclareLaunchArgument('launch_hawks', default_value=launch_hawks,
                                             description='Launch Hawk Cameras')
    launch_hesai = LaunchConfiguration('launch_hesai', default=True)
    launch_hesai_arg = DeclareLaunchArgument('launch_hesai', default_value=launch_hesai,
                                             description='Launch Hesai 3D Lidar')
    launch_rplidars = LaunchConfiguration('launch_rplidars', default=True)
    launch_rplidars_arg = DeclareLaunchArgument('launch_rplidars', default_value=launch_rplidars,
                                                description='Launch rplidars')
    launch_segway = LaunchConfiguration('launch_segway', default=True)
    launch_segway_arg = DeclareLaunchArgument('launch_segway', default_value=launch_segway,
                                              description='Launch Segway Driver')
    launch_nvblox_pipeline = LaunchConfiguration(
        'launch_nvblox_pipeline', default=False)
    launch_nvblox_pipeline_arg = DeclareLaunchArgument(
        'launch_nvblox_pipeline', default_value=launch_nvblox_pipeline,
        description='Launch Hawk Camera + ESS Depth + Isaac ROS Nvblox')
    launch_mission_client = LaunchConfiguration(
        'launch_mission_client', default=False)
    launch_mission_client_arg = DeclareLaunchArgument(
        'launch_mission_client',
        default_value=launch_mission_client,
        description='Launch Isaac ROS Mission Client')
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        carter_navigation_params_dir, 'carter_nav2.yaml'))
    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_file,
                                            description='Nav2 params file')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Running Simulation?')
    launch_args = [launch_owls_arg,
                   launch_hawks_arg,
                   launch_hesai_arg,
                   launch_rplidars_arg,
                   launch_segway_arg,
                   launch_nvblox_pipeline_arg,
                   launch_mission_client_arg,
                   params_file_arg,
                   use_sim_time_arg
                   ]

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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_toolbox_launch_dir, '/online_async_launch.py']),
        launch_arguments={'slam_params_file': params_file,
                          'use_sim_time': use_sim_time}.items()
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, '/navigation_launch.py']),
        launch_arguments={'params_file': params_file,
                          'use_composition': 'True',
                          'use_sim_time': use_sim_time,
                          'container_name': 'carter_container'}.items()
    )

    segway_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/segway.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_segway', 'True')
    )

    mission_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [mission_client_launch_dir, '/isaac_ros_vda5050_client.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_mission_client', 'True')
    )

    carter_container = Node(
        name='carter_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_file],
        output='screen')

    correlated_timestamp_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/correlated_timestamp_driver.launch.py'])
    )

    hawks_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/4_hawks.launch.py']),
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

    nvblox_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [carter_navigation_launch_include_dir, '/front_hawk_ess_nvblox.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_nvblox_pipeline', 'True')
    )

    ld = LaunchDescription(launch_args + [
        urdf_static_transform_publisher_launch,
        foxglove_bridge_launch,
        joystick_launch,
        twist_mux_launch,
        slam_toolbox_launch,
        nav2_navigation_launch,
        segway_launch,
        mission_client_launch,
        carter_container,
        correlated_timestamp_driver_launch,
        hawks_launch,
        hesai_launch,
        rplidars_launch,
        nvblox_pipeline_launch
    ])

    return ld
