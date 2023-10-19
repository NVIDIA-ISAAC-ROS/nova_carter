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
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    owl_front_node = ComposableNode(
        name='owl_front_node',
        package='isaac_ros_owl',
        plugin='nvidia::isaac_ros::owl::OwlNode',
        namespace='owl_front',
        remappings=[
            ('/owl_front/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'camera_id': 0,
                    'module_id': 0,
                     'camera_link_frame_name': 'front_fisheye_camera',
                     'optical_frame_name': 'front_fisheye_camera_optical'}]
    )

    owl_back_node = ComposableNode(
        name='owl_back_node',
        package='isaac_ros_owl',
        plugin='nvidia::isaac_ros::owl::OwlNode',
        namespace='owl_back',
        remappings=[
            ('/owl_back/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'camera_id': 1,
                    'module_id': 1,
                     'camera_link_frame_name': 'rear_fisheye_camera',
                     'optical_frame_name': 'rear_fisheye_camera_optical'}]
    )

    owl_left_node = ComposableNode(
        name='owl_left_node',
        package='isaac_ros_owl',
        plugin='nvidia::isaac_ros::owl::OwlNode',
        namespace='owl_left',
        remappings=[
            ('/owl_left/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'camera_id': 4,
                    'module_id': 3,
                     'camera_link_frame_name': 'left_fisheye_camera',
                     'optical_frame_name': 'left_fisheye_camera_optical'
                    }]
    )

    owl_right_node = ComposableNode(
        name='owl_right_node',
        package='isaac_ros_owl',
        plugin='nvidia::isaac_ros::owl::OwlNode',
        namespace='owl_right',
        remappings=[
            ('/owl_right/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'camera_id': 5,
                    'module_id': 4,
                     'camera_link_frame_name': 'right_fisheye_camera',
                     'optical_frame_name': 'right_fisheye_camera_optical'
                    }]
    )

    front_resize_node = ComposableNode(
        name='front_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='owl_front',
        parameters=[{
            'output_width': 192,
            'output_height': 120,
        }],
        remappings=[
            ('image', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camerainfo_resized')
        ]
    )

    back_resize_node = ComposableNode(
        name='back_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='owl_back',
        parameters=[{
            'output_width': 192,
            'output_height': 120,
        }],
        remappings=[
            ('image', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camerainfo_resized')
        ]
    )

    left_resize_node = ComposableNode(
        name='left_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='owl_left',
        parameters=[{
            'output_width': 192,
            'output_height': 120,
        }],
        remappings=[
            ('image', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camerainfo_resized')
        ]
    )

    right_resize_node = ComposableNode(
        name='right_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='owl_right',
        parameters=[{
            'output_width': 192,
            'output_height': 120,
        }],
        remappings=[
            ('image', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camerainfo_resized')
        ]
    )

    load_owls = LoadComposableNodes(
        target_container='carter_container',
        composable_node_descriptions=[
            owl_front_node,
            front_resize_node,
            owl_back_node,
            back_resize_node,
            owl_left_node,
            left_resize_node,
            owl_right_node,
            right_resize_node
        ]
    )

    return LaunchDescription([load_owls])
