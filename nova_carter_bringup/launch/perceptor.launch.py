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

from typing import List


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode', 'real_world', cli=True, choices=[
                 'real_world', 'simulation', 'rosbag'])
    args.add_arg('rosbag', 'None', cli=True)
    args.add_arg('sim', False, cli=True)
    args.add_arg('enable_3d_lidar', False, cli=True)
    args.add_arg('enabled_2d_lidars', '', cli=True)
    args.add_arg('enabled_fisheye_cameras', '', cli=True)
    args.add_arg('enable_wheel_odometry', False, cli=True)
    args.add_arg('enable_cuvslam', True, cli=True)
    args.add_arg('stereo_camera_configuration',
                 default='front_left_right_configuration',
                 choices=[
                     'front_configuration',
                     'front_people_configuration',
                     'front_left_right_configuration',
                     'front_back_left_right_vo_configuration',
                     'front_back_left_right_vgl_configuration',
                     'front_left_right_ess_full_configuration',
                 ],
                 cli=True)
    args.add_arg('use_foxglove_whitelist', True, cli=True)
    args.add_arg('type_negotiation_duration_s',
                 lu.get_default_negotiation_time(), cli=True)
    args.add_arg('nvblox_param_filename',
                 'params/nvblox_perceptor.yaml', cli=True)
    args.add_arg('nvblox_after_shutdown_map_save_path', '', cli=True)
    args.add_arg('sim_global_frame', 'odom_vslam', cli=True)
    args.add_arg('global_frame', 'map', cli=True)
    args.add_arg('vslam_odom_frame', 'odom', cli=True)
    args.add_arg('vslam_map_frame', 'map', cli=True)

    is_sim = lu.is_equal(args.mode, 'simulation')
    is_real_world = lu.is_equal(args.mode, 'real_world')

    actions = args.get_launch_actions()
    actions.append(SetParameter('type_negotiation_duration_s',
                   args.type_negotiation_duration_s))
    actions.append(SetParameter(
        'use_sim_time',
        True,
        condition=UnlessCondition(is_real_world),
    ))

    # Add the perceptor (note that this also includes the hardware abstraction layer for now).
    # NOTE: If running in sim mode we do mapping in another frame to not clash with ground truth
    # coming out of Isaac Sim.
    # NOTE: If running in sim mode we are setting the reliability policy of vslam to reliable
    # as we were experiencing issues with the best effort policy
    # (i.e. not able to synchronize input messages due to dropped images).
    global_frame = lu.if_else_substitution(
        is_sim, args.sim_global_frame, args.global_frame)
    vslam_odom_frame = lu.if_else_substitution(
        is_sim, args.sim_global_frame, args.vslam_odom_frame)

    vslam_image_qos = lu.if_else_substitution(is_sim, 'DEFAULT', 'SENSOR_DATA')
    # We disable cuVSLAM if a user requests wheel odometry
    disable_cuvslam = OrSubstitution(args.enable_wheel_odometry,
                                     NotSubstitution(args.enable_cuvslam))
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/include/perceptor_include.launch.py',
            launch_arguments={
                'global_frame': global_frame,
                'vslam_image_qos': vslam_image_qos,
                'vslam_odom_frame': vslam_odom_frame,
                'vslam_map_frame': args.vslam_map_frame,
                'disable_cuvslam': disable_cuvslam,
                'nvblox_param_filename': args.nvblox_param_filename,
                'nvblox_after_shutdown_map_save_path': args.nvblox_after_shutdown_map_save_path,
                'is_sim': is_sim,
                'enable_3d_lidar': args.enable_3d_lidar,
            },
        ))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/tools/visualization.launch.py',
            launch_arguments={
                'use_foxglove_whitelist': args.use_foxglove_whitelist},
        ))
    actions.append(lu.component_container('nova_container'))

    return LaunchDescription(actions)
