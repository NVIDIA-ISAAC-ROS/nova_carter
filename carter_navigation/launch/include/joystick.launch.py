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

from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_config_arg = DeclareLaunchArgument('joy_config', default_value='ps5')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0')
    joy_vel = LaunchConfiguration('joy_vel')
    joy_vel_arg = DeclareLaunchArgument('joy_vel', default_value='joy/cmd_vel')
    config_filepath = LaunchConfiguration('config_filepath')
    config_filepath_arg = DeclareLaunchArgument('config_filepath', default_value=[
        TextSubstitution(text=os.path.join(
            get_package_share_directory('carter_navigation'), 'params', '')),
        joy_config, TextSubstitution(text='.config.yaml')])

    joy_linux_node = Node(
        package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node', parameters=[config_filepath],
        remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},
    )

    return LaunchDescription([
        joy_config_arg,
        joy_dev_arg,
        joy_vel_arg,
        config_filepath_arg,
        joy_linux_node,
        teleop_twist_joy_node
    ])
