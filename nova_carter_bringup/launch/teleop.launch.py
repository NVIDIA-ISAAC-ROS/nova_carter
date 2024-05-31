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
    args.add_arg('mode', 'real_world', choices=['real_world', 'simulation', 'rosbag'], cli=True)
    args.add_arg('rosbag', 'None', cli=True)
    args.add_arg('enabled_stereo_cameras',
                 'front_stereo_camera,left_stereo_camera,right_stereo_camera',
                 cli=True)
    args.add_arg('enabled_fisheye_cameras', 'front_fisheye_camera', cli=True)
    args.add_arg('enabled_2d_lidars', 'front_2d_lidar,back_2d_lidar', cli=True)
    args.add_arg('enable_3d_lidar', True, cli=True)
    args.add_arg('enable_wheel_odometry', True, cli=True)
    args.add_arg('type_negotiation_duration_s', 5, cli=True)

    actions = args.get_launch_actions()
    actions.append(SetParameter('type_negotiation_duration_s', args.type_negotiation_duration_s))
    actions.append(
        SetParameter(
            'use_sim_time',
            True,
            condition=IfCondition(lu.is_equal(args.mode, 'simulation')),
        ))

    actions.append(
        lu.include('nova_carter_bringup',
                   'launch/include/hardware_abstraction_layer_include.launch.py'))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/tools/visualization.launch.py',
            launch_arguments={'use_foxglove_whitelist': False},
        ))

    actions.append(lu.component_container('nova_container'))

    return LaunchDescription(actions)
