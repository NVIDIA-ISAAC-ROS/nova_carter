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
    args.add_arg('mode', choices=['real_world', 'simulation', 'rosbag'])

    is_real_world = lu.is_equal(args.mode, 'real_world')
    is_rosbag = lu.is_equal(args.mode, 'rosbag')

    actions = args.get_launch_actions()

    actions.append(
        lu.include(
            'nova_carter_description',
            'launch/nova_carter_description.launch.py',
            condition=UnlessCondition(is_rosbag),
        ))
    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/drivers/nova_sensor_abstraction_layer.launch.py',
        ))
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/include/teleop_include.launch.py',
            condition=IfCondition(is_real_world),
        ))

    return LaunchDescription(actions)