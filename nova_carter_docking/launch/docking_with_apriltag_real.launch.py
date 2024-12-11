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

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='True',
            description='Launch RViz if set to True'),
        DeclareLaunchArgument(
            'image',
            default_value='/front_stereo_camera/left/image_rect',
            description='Image topic'),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/front_stereo_camera/left/camera_info_rect',
            description='Camera info topic'),
        DeclareLaunchArgument(
            'stereo_camera_configuration',
            default_value='front_driver_rectify',
            description='Other choices: front_left_right_configuration, front_driver_rectify'),
        DeclareLaunchArgument(
            'map_yaml_path',
            default_value='maps/nvidia_galileo.yaml',
            description='Map yaml file path'),
    ]

    launch_rviz = LaunchConfiguration('launch_rviz')
    image = LaunchConfiguration('image')
    camera_info = LaunchConfiguration('camera_info')
    stereo_camera_configuration = LaunchConfiguration('stereo_camera_configuration')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    nova_carter_dock_launch_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'launch')
    nova_carter_bringup_launch_dir = os.path.join(
        get_package_share_directory('nova_carter_bringup'), 'launch')
    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'params')

    params_file = os.path.join(
        nova_carter_dock_params_dir, 'nova_carter_docking.yaml')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nova_carter_bringup_launch_dir, 'navigation.launch.py')),
        launch_arguments={
            'enable_nvblox_costmap': 'False',
            'stereo_camera_configuration': stereo_camera_configuration,
            'map_yaml_path': map_yaml_path
        }.items()
    )

    dock_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nova_carter_dock_launch_dir, 'isaac_apriltag_detection_pipeline.launch.py')),
        launch_arguments={
            'launch_rviz': launch_rviz,
            'image': image,
            'camera_info': camera_info,
        }.items()
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    return launch.LaunchDescription(launch_args + [
        navigation_launch,
        dock_detection_launch,
        docking_server,
        lifecycle_manager
    ])
