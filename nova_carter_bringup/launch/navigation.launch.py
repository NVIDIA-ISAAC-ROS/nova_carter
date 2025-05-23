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
import isaac_ros_launch_utils.all_types as lut


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode', 'real_world', cli=True, choices=['real_world', 'simulation', 'rosbag'])
    args.add_arg('rosbag', 'None', cli=True)
    args.add_arg('navigation_parameters_path',
                 lu.get_path('nova_carter_navigation', 'params/nova_carter_navigation.yaml'),
                 cli=True)
    args.add_arg('map_yaml_path', 'None', cli=True)
    args.add_arg('enable_navigation', True, cli=True)
    args.add_arg('enable_mission_client', False, cli=True)
    args.add_arg('enable_docking', False, cli=True)
    args.add_arg('enable_3d_lidar_costmap', False, cli=True)
    args.add_arg('enable_2d_lidar_costmap', True, cli=True)
    args.add_arg('enable_nvblox_costmap', True, cli=True)
    args.add_arg('enable_wheel_odometry', True, cli=True)
    args.add_arg('enable_3d_lidar_localization', True, cli=True)
    args.add_arg('enable_visual_localization', False, cli=True)
    args.add_arg('stereo_camera_configuration',
                 default='front_configuration',
                 choices=['no_cameras', 'front_configuration',
                          'front_left_right_configuration', 'front_driver_rectify',
                          'front_left_right_vslam_configuration'],
                 cli=True)
    args.add_arg('enabled_fisheye_cameras', '', cli=True)
    args.add_arg('use_foxglove_whitelist', True, cli=True)
    args.add_arg('type_negotiation_duration_s', lu.get_default_negotiation_time(), cli=True)

    is_sim = lu.is_equal(args.mode, 'simulation')
    enable_global_navigation = lu.is_valid(args.map_yaml_path)

    actions = args.get_launch_actions()
    actions.append(lut.SetParameter('type_negotiation_duration_s',
                   args.type_negotiation_duration_s))
    actions.append(
        lu.log_info(['Using type negotiation duration: ', args.type_negotiation_duration_s]))
    actions.append(lut.SetParameter('use_sim_time', True, condition=lut.IfCondition(is_sim)))

    # NOTE: If running in sim mode we do mapping in another frame to not clash with ground truth
    # coming out of Isaac Sim.
    # Set global frame to odom_vslam when:
    # - running local navigation in Isaac Sim (i.e. no global map passed)
    run_local_navigation_in_sim = lu.AndSubstitution(
        lu.is_false(enable_global_navigation), lu.is_true(is_sim))
    global_frame = lu.if_else_substitution(run_local_navigation_in_sim, 'odom_vslam', 'odom')

    # NOTE: If running in sim mode we are setting the reliability policy of vslam to reliable
    # as we were experiencing issues with the best effort policy
    # (i.e. not able to synchronize input messages due to dropped images).
    vslam_image_qos = lu.if_else_substitution(is_sim, 'DEFAULT', 'SENSOR_DATA')

    # Add the navigation.
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/include/navigation_include.launch.py',
            launch_arguments={
                'global_frame': global_frame,
                'vslam_image_qos': vslam_image_qos,
            },
        ))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/tools/visualization.launch.py',
            launch_arguments={'use_foxglove_whitelist': args.use_foxglove_whitelist},
        ))

    # We use a negative niceness to increase the priority.
    actions.append(lu.component_container('nova_container', prefix='nice -n -10'))
    # A separate container is needed for Nav2 because ROS2 has a race condition for actions when
    # using non-isolated containers.
    # Nav container has to be added last, else Nav2 parameters are not picked up properly.
    actions.append(lu.component_container('navigation_container', container_type='isolated'))

    return lut.LaunchDescription(actions)
