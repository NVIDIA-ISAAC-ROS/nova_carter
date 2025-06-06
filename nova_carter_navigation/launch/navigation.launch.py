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

# flake8: noqa: F403,F405
import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import *


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('enable_mission_client')
    args.add_arg('enable_docking')
    args.add_arg('navigation_parameters_path')
    args.add_arg('navigation_container_name', 'navigation_container')
    args.add_arg('use_sim_time', False)

    actions = args.get_launch_actions()
    actions.append(
        lu.include(
            'nav2_bringup',
            'launch/navigation_launch.py',
            launch_arguments={
                # The parameters file here does not work correctly when using
                # composition. In addition to here it also has to be loaded with
                # `SetParameterFromFile` before the corresponding
                # component_container is added to the launch description.
                'params_file': args.navigation_parameters_path,
                'container_name': args.navigation_container_name,
                # NOTE: nav2 wants this parameter as a capitalized string.
                'use_composition': 'True',
                'use_sim_time': args.use_sim_time,
            },
        ))

    actions.append(
        lu.include(
            'isaac_ros_vda5050_nav2_client_bringup',
            'launch/isaac_ros_vda5050_client.launch.py',
            launch_arguments={
                'docking_server_enabled': args.enable_docking,
            },
            condition=IfCondition(args.enable_mission_client),
        ))

    switch_node = ComposableNode(
        package='custom_nitros_image',
        plugin='nvidia::isaac_ros::custom_nitros_image::NitrosImageSwitchNode',
        name='switch_node',
        remappings=[
            ('image', '/front_stereo_camera/left/image_rect'),
            ('camera_info', '/front_stereo_camera/left/camera_info_rect'),
            ('switched_image', '/front_stereo_camera/left/switched_image_rect'),
            ('switched_camera_info', '/front_stereo_camera/left/switched_camera_info_rect'),
        ],
        parameters=[{
            'initial_switch_state': False
        }],
        condition=IfCondition(args.enable_docking),
    )
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[('image', '/front_stereo_camera/left/switched_image_rect'),
                    ('camera_info', '/front_stereo_camera/left/switched_camera_info_rect')],
        condition=IfCondition(args.enable_docking),
        parameters=[{
            'size': 0.1524,  # 6 inches
            'max_tags': 4,
            'tile_size': 4,
        }],
    )
    load_nodes = lu.load_composable_nodes(
        args.navigation_container_name,
        [switch_node, apriltag_node],
        condition=IfCondition(args.enable_docking),
    )
    actions.append(load_nodes)

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[str(lu.get_path('nova_carter_docking', 'params/nova_carter_docking.yaml'))],
        condition=IfCondition(args.enable_docking),
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'autostart': True
        }, {
            'node_names': ['docking_server']
        }],
        condition=IfCondition(args.enable_docking),
    )

    dock_pose_publisher = Node(
        package='nova_carter_docking',
        executable='dock_pose_publisher',
        name='dock_pose_publisher',
        parameters=[{
            'use_first_detection': True,
            'dock_tag_id': 586,
        }],
        condition=IfCondition(args.enable_docking),
    )
    actions.extend([docking_server, lifecycle_manager, dock_pose_publisher])

    return LaunchDescription(actions)
