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

import pathlib
import unittest

import ament_index_python.packages
import isaac_ros_launch_utils as lu
from isaac_ros_test import IsaacROSBaseTest
from launch import LaunchDescription
import launch_testing
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import DistanceMapSlice
import pytest
from sensor_msgs.msg import Image


BAG_NAME = 'r2b_galileo_compressed'
TIMEOUT = 120


@pytest.mark.rostest
def generate_test_description():
    # Launch perceptor configured for a single hawk.
    bag_path = pathlib.Path(
        ament_index_python.packages.get_package_share_directory(
            'isaac_ros_r2b_galileo')) / 'data/r2b_galileo'
    actions = []
    actions.append(
        lu.include('nova_carter_bringup',
                   'launch/perceptor.launch.py',
                   launch_arguments={
                       'stereo_camera_configuration': 'front_left_right_configuration',
                       'rosbag': bag_path,
                       'run_rviz': False,
                       'run_foxglove': False,
                       'type_negotiation_duration_s': 30,
                       'mode': 'rosbag',
                   }))
    # Required for ROS launch testing.
    actions.append(launch_testing.util.KeepAliveProc())
    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions)


# NOTE(dtingdahl) Mark this as expectedFailure while invesetigating its behavior in CI. We ensure
# at the end that the test always fail (which will be reported as a success to the test system)
@unittest.expectedFailure
class IsaacROSNvBloxTest(IsaacROSBaseTest):
    filepath = pathlib.Path(__file__).parent

    def test_nova_carter_perceptor(self):
        received_messages = {}
        # Note: Formatting of py-logging gets reset inside launch tests so we use print instead.
        print(__file__ + ': Creating subscribers')
        subs = self.create_logging_subscribers(
            [('/nvblox_node/static_map_slice', DistanceMapSlice),
             ('/front_stereo_camera/depth', Image), ('/left_stereo_camera/depth', Image),
             ('/right_stereo_camera/depth', Image), ('/visual_slam/tracking/odometry', Odometry)],
            received_messages,
            use_namespace_lookup=False,
            accept_multiple_messages=True)

        try:
            print(__file__ + ': Spinning')
            self.spin_node_until_messages_received(received_messages, TIMEOUT)
            print(__file__ + ': Spinning completed')
            self.assert_messages_received(received_messages)

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]

        print('Success')
        assert False
