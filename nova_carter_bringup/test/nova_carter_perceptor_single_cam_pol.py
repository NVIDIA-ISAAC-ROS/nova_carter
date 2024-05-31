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

import os
import pathlib

import isaac_ros_launch_utils as lu
from isaac_ros_test import IsaacROSBaseTest
from launch import LaunchDescription
import launch_testing
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import DistanceMapSlice, Mesh
import pytest
from sensor_msgs.msg import Image

BAG_NAME = '2024_01_31_with_obstacles_no_lidar_30s'
TIMEOUT = 120


@pytest.mark.rostest
def generate_test_description():
    # Launch perceptor configured for a single hawk.
    bag_path = lu.get_path('nova_carter_example_data', os.path.join('data', BAG_NAME))
    actions = []
    actions.append(
        lu.include('nova_carter_bringup',
                   'launch/perceptor.launch.py',
                   launch_arguments={
                       'stereo_camera_configuration': 'front_configuration',
                       'rosbag': bag_path,
                       'run_rviz': False,
                       'run_foxglove': False,
                       'type_negotiation_duration_s': 30,
                   }))
    # Required for ROS launch testing.
    actions.append(launch_testing.util.KeepAliveProc())
    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions)


class IsaacROSNvBloxTest(IsaacROSBaseTest):
    filepath = pathlib.Path(__file__).parent

    def test_nova_carter_perceptor(self):
        received_messages = {}

        subs = self.create_logging_subscribers(
            [('/nvblox_node/mesh', Mesh), ('/nvblox_node/static_map_slice', DistanceMapSlice),
             ('/front_stereo_camera/depth', Image), ('/visual_slam/tracking/odometry', Odometry)],
            received_messages,
            use_namespace_lookup=False,
            accept_multiple_messages=True)

        try:
            self.spin_node_until_messages_received(received_messages, TIMEOUT)
            self.assert_messages_received(received_messages)

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
