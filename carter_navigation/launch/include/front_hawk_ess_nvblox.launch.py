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
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Check if ESS model is available
    models_dir_path = os.path.dirname(
        os.path.realpath(__file__))+'/../../models'
    engine_file_path = models_dir_path+'/ess.engine'
    if not os.path.isfile(engine_file_path):
        raise Exception(
            f'ESS engine file not found at : {engine_file_path}.'
        )

    hawk_front_node = ComposableNode(
        name='hawk_front_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace='hawk_front',
        parameters=[{'module_id': 5,
                     'camera_link_frame_name': 'front_stereo_camera',
                     'left_optical_frame_name': 'front_stereo_camera_left_optical',
                     'right_optical_frame_name': 'front_stereo_camera_right_optical', }],
        remappings=[
            ('/hawk_front/correlated_timestamp', '/correlated_timestamp')
        ]
    )

    left_rectify_node = ComposableNode(
        name='left_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        namespace='hawk_front',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200
        }],
        remappings=[
            ('image_raw', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('image_rect', 'left/image_rect'),
            ('camera_info_rect', 'left/camera_info_rect')
        ]
    )

    right_rectify_node = ComposableNode(
        name='right_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        namespace='hawk_front',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
        }],
        remappings=[
            ('image_raw', 'right/image_raw'),
            ('camera_info', 'right/camerainfo'),
            ('image_rect', 'right/image_rect'),
            ('camera_info_rect', 'right/camera_info_rect')
        ]
    )

    # Adding resize before ESS since resized camera info is required
    # by the depth_to_pointcloud_node
    left_resize_node = ComposableNode(
        name='left_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='hawk_front',
        parameters=[{
            'output_width': 960,
            'output_height': 576,
        }],
        remappings=[
            ('image', 'left/image_rect'),
            ('camera_info', 'left/camera_info_rect'),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camera_info_resized')
        ]
    )

    ess_node = ComposableNode(
        name='ess_node',
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
        namespace='hawk_front',
        parameters=[{'engine_file_path': engine_file_path,
                     'threshold': 0.92
                     }],
        remappings=[
            ('left/image_rect', 'left/image_rect'),
            ('right/image_rect', 'right/image_rect'),
            ('left/camera_info', 'left/camera_info_rect'),
            ('right/camera_info', 'right/camera_info_rect')
        ]
    )

    disparity_to_depth_node = ComposableNode(
        name='disparity_to_depth',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
        namespace='hawk_front'
    )

    depth_to_pointcloud_node = ComposableNode(
        name='point_cloud_xyz_node',
        package='isaac_ros_depth_image_proc',
        plugin='nvidia::isaac_ros::depth_image_proc::PointCloudXyzNode',
        namespace='hawk_front',
        parameters=[{
                'skip': 21
        }],
        remappings=[
            ('image_rect', 'depth'),
            ('camera_info', 'left/camera_info_resized'),
            ('points', 'ess_points')
        ]
    )

    nvblox_config = os.path.join(
        get_package_share_directory('carter_navigation'),
        'params',
        'nvblox_carter.yaml'
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        namespace='hawk_front',
        remappings=[('depth/image', 'depth'),
                    ('depth/camera_info', 'left/camera_info_resized'),
                    ('color/image', 'left/image_resized'),
                    ('color/camera_info', 'left/camera_info_resized')],
        parameters=[nvblox_config]
    )

    load_hawk_ess_nvblox = LoadComposableNodes(
        target_container='carter_container',
        composable_node_descriptions=[
            hawk_front_node,
            left_rectify_node,
            right_rectify_node,
            left_resize_node,
            ess_node,
            disparity_to_depth_node,
            depth_to_pointcloud_node,
            nvblox_node
        ]
    )

    return LaunchDescription([load_hawk_ess_nvblox])
