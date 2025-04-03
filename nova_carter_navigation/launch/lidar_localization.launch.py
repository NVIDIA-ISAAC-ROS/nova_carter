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
    args.add_arg('map_yaml_path')
    args.add_arg('navigation_parameters_path')
    args.add_arg('container_name', 'navigation_container')
    args.add_arg('type_negotiation_duration_s')
    args.add_arg('set_init_pose', False)

    actions = args.get_launch_actions()

    actions.append(lu.log_info('Enabling ROS 2 Nav2 lidar localization.'))
    actions.append(
        lu.include(
            'nav2_bringup',
            'launch/localization_launch.py',
            delay=10.0,
            launch_arguments={
                'map': args.map_yaml_path,
                'params_file': args.navigation_parameters_path,
                'use_composition': 'True',
                'container_name': 'nova_container',
            },
        ))

    actions.append(
        lu.log_info(
            'Enabling occupancy grid localizer.',
            condition=IfCondition(lu.NotSubstitution(args.set_init_pose))
        ))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/algorithms/occupancy_grid_localizer.launch.py',
            launch_arguments={'map_yaml_path': args.map_yaml_path},
            condition=IfCondition(lu.NotSubstitution(args.set_init_pose)),
        ))

    actions.append(
        lu.service_call(
            service='/trigger_grid_search_localization ',
            type='std_srvs/srv/Empty ',
            content='"{}"',
            delay=PythonExpression([args.type_negotiation_duration_s, ' + 15.0'])
        ))

    return LaunchDescription(actions)
