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
    args.add_arg('lidar_mapping_parameters_path',
                 lu.get_path('nova_carter_navigation', 'params/nova_carter_navigation.yaml'),
                 cli=True)
    actions = args.get_launch_actions()
    actions.append(lu.assert_path_exists(args.lidar_mapping_parameters_path))
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/teleop.launch.py',
            launch_arguments={
                'enabled_fisheye_cameras': 'none',
                'enabled_stereo_cameras': 'none',
                'enabled_2d_lidars': 'none',
                'enable_3d_lidar': True,
                'enable_wheel_odometry': True,
            },
        ))

    actions.append(
        lu.include(
            'slam_toolbox',
            'launch/online_async_launch.py',
            launch_arguments={
                'slam_params_file': args.lidar_mapping_parameters_path,
            },
        ))

    actions.append(lu.component_container('nova_container'))

    return LaunchDescription(actions)
