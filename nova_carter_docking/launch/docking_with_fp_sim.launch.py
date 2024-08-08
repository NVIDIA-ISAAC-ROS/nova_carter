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


ISAAC_ROS_ASSETS_PATH = os.path.join(
    os.environ['ISAAC_ROS_WS'], 'isaac_ros_assets')
ISAAC_ROS_MODELS_PATH = os.path.join(ISAAC_ROS_ASSETS_PATH, 'models')
ISAAC_ROS_FP_MESHES_PATH = os.path.join(ISAAC_ROS_ASSETS_PATH,
                                        'isaac_ros_foundationpose')
SYNTHETICA_DETR_MODELS_PATH = os.path.join(
    ISAAC_ROS_MODELS_PATH, 'synthetica_detr')
RTDETR_ENGINE_PATH = os.path.join(
    SYNTHETICA_DETR_MODELS_PATH, 'sdetr_amr.plan')
MESH_OBJ_PATH = os.path.join(ISAAC_ROS_FP_MESHES_PATH,
                             'dock', 'dock.obj')
MESH_TEX_PATH = os.path.join(ISAAC_ROS_FP_MESHES_PATH,
                             'dock', 'materials', 'textures', 'baked_mesh_tex0.png')


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
            'mesh_file_path',
            default_value=MESH_OBJ_PATH,
            description='The absolute file path to the mesh file'),
        DeclareLaunchArgument(
            'texture_path',
            default_value=MESH_TEX_PATH,
            description='The absolute file path to the texture map'),
        DeclareLaunchArgument(
            'rt_detr_engine_file_path',
            default_value=RTDETR_ENGINE_PATH,
            description='The absolute file path to the RT-DETR TensorRT engine file'),
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
    mesh_file_path = LaunchConfiguration('mesh_file_path')
    texture_path = LaunchConfiguration('texture_path')
    rt_detr_engine_file_path = LaunchConfiguration('rt_detr_engine_file_path')

    param_substitutions = {
        'x': init_pose_x,
        'y': init_pose_y,
        'yaw': init_pose_yaw
    }

    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    foundationpose_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_foundationpose'), 'launch')
    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'params')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    params_file = os.path.join(
        nova_carter_dock_params_dir, 'nova_carter_docking_fp.yaml')

    foundationpose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([foundationpose_launch_dir,
                                       '/isaac_ros_foundationpose_isaac_sim_tracking.launch.py']),
        launch_arguments={'launch_rviz': launch_rviz,
                          'mesh_file_path': mesh_file_path,
                          'texture_path': texture_path,
                          'rt_detr_engine_file_path': rt_detr_engine_file_path}.items(),
    )

    fp_dock_pose_publisher = Node(
        package='nova_carter_docking',
        executable='fp_dock_pose_publisher.py',
        name='fp_dock_pose_publisher'
    )

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

    return launch.LaunchDescription(launch_args + [
        foundationpose_launch,
        fp_dock_pose_publisher,
        nav2_bringup_launch,
        docking_server,
        lifecycle_manager
    ])
