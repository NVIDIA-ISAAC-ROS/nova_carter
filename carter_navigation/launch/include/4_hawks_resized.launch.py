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
    hawk_front_node = ComposableNode(
        name='hawk_front_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace='hawk_front',
        remappings=[
            ('/hawk_front/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'module_id': 5,
                     'camera_link_frame_name': 'front_stereo_camera',
                     'left_optical_frame_name': 'front_stereo_camera_left_optical',
                     'right_optical_frame_name': 'front_stereo_camera_right_optical'}]
    )

    hawk_back_node = ComposableNode(
        name='hawk_back_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace='hawk_back',
        remappings=[
            ('/hawk_back/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'module_id': 6,
                     'camera_link_frame_name': 'rear_stereo_camera',
                     'left_optical_frame_name': 'rear_stereo_camera_left_optical',
                     'right_optical_frame_name': 'rear_stereo_camera_right_optical'}]
    )

    hawk_left_node = ComposableNode(
        name='hawk_left_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace='hawk_left',
        remappings=[
            ('/hawk_left/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'module_id': 7,
                     'camera_link_frame_name': 'left_stereo_camera',
                     'left_optical_frame_name': 'left_stereo_camera_left_optical',
                     'right_optical_frame_name': 'left_stereo_camera_right_optical'}]
    )

    hawk_right_node = ComposableNode(
        name='hawk_right_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace='hawk_right',
        remappings=[
            ('/hawk_right/correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=[{'module_id': 2,
                     'camera_link_frame_name': 'right_stereo_camera',
                     'left_optical_frame_name': 'right_stereo_camera_left_optical',
                     'right_optical_frame_name': 'right_stereo_camera_right_optical'}]
    )

    front_resize_node = ComposableNode(
        name='front_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='hawk_front',
        parameters=[{
            'output_width': 96,
            'output_height': 60,
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
        namespace='hawk_back',
        parameters=[{
            'output_width': 96,
            'output_height': 60,
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
        namespace='hawk_left',
        parameters=[{
            'output_width': 96,
            'output_height': 60,
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
        namespace='hawk_right',
        parameters=[{
            'output_width': 96,
            'output_height': 60,
        }],
        remappings=[
            ('image', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camerainfo_resized')
        ]
    )

    load_hawks = LoadComposableNodes(
        target_container='carter_container',
        composable_node_descriptions=[
            hawk_front_node,
            front_resize_node,
            hawk_back_node,
            back_resize_node,
            hawk_left_node,
            left_resize_node,
            hawk_right_node,
            right_resize_node
        ]
    )

    return LaunchDescription([load_hawks])
