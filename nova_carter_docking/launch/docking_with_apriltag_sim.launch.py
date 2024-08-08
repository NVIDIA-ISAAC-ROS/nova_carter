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
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for ROS nodes in this launch script'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use composed Nav2 bringup'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Omniverse Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'init_pose_x',
            default_value='0.0',
            description='Initial position X coordinate'),
        DeclareLaunchArgument(
            'init_pose_y',
            default_value='0.0',
            description='Initial position Y coordinate'),
        DeclareLaunchArgument(
            'init_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory(
                    'isaac_ros_vda5050_nav2_client_bringup'),
                'maps', 'carter_warehouse_navigation.yaml'
            ),
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(
                get_package_share_directory(
                    'isaac_ros_vda5050_nav2_client_bringup'),
                'config', 'carter_navigation_params.yaml'
            ),
            description='Full path to navigation param file to load'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='True',
            description='Launch RViz if set to True'),
        DeclareLaunchArgument(
            'image',
            default_value='/front_stereo_camera/left/image_rect_color',
            description='Image topic'),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/front_stereo_camera/left/camera_info',
            description='Camera info topic'),
    ]
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose_x = LaunchConfiguration('init_pose_x', default=0.0)
    init_pose_y = LaunchConfiguration('init_pose_y', default=0.0)
    init_pose_yaw = LaunchConfiguration('init_pose_yaw', default=0.0)
    map_dir = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('nav_params_file',)
    launch_rviz = LaunchConfiguration('launch_rviz')
    image = LaunchConfiguration('image')
    camera_info = LaunchConfiguration('camera_info')
    nova_carter_dock_launch_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'launch')

    param_substitutions = {
        'x': init_pose_x,
        'y': init_pose_y,
        'yaw': init_pose_yaw
    }

    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'params')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    params_file = os.path.join(
        nova_carter_dock_params_dir, 'nova_carter_docking.yaml')

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_composition': use_composition,
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
        }.items(),
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

    return launch.LaunchDescription(launch_args + [
        dock_detection_launch,
        nav2_bringup_launch,
        docking_server,
        lifecycle_manager
    ])
