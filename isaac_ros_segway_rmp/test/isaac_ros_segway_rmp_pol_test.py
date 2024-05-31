# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import select
import subprocess
import time

from isaac_ros_nova_interfaces.msg import EncoderTicks
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import BatteryState, Imu


@pytest.mark.rostest
def check_candump_output():
    proc = subprocess.Popen(['candump', 'can0'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    start_time = time.time()
    timeout = 3  # seconds

    while time.time() - start_time < timeout:
        readable, _, _ = select.select([proc.stdout], [], [], timeout)
        if readable:
            output = proc.stdout.readline()
            if output:
                proc.terminate()
                return True

    proc.terminate()
    return False


def generate_test_description():
    # Ping segway wheelbase using the executable to check if it is present,
    # before trying to test the node
    IsaacROSSegwayRMPPOLTest.skip_test = False
    if (check_candump_output()):
        """Generate launch description with all ROS 2 nodes for testing."""
        segway_rmp_node = ComposableNode(
            package='isaac_ros_segway_rmp',
            plugin='nvidia::isaac_ros::segway_rmp::SegwayRMPNode',
            name='segway_rmp',
            namespace=IsaacROSSegwayRMPPOLTest.generate_namespace()
        )

        segway_rmp_container = ComposableNodeContainer(
            package='rclcpp_components',
            name='segway_rmp_container',
            namespace='',
            executable='component_container_mt',
            composable_node_descriptions=[
                segway_rmp_node,
            ],
            output='screen'
        )
        return IsaacROSSegwayRMPPOLTest.generate_test_description([
            segway_rmp_container
        ])
    else:
        IsaacROSSegwayRMPPOLTest.skip_test = True
        return IsaacROSSegwayRMPPOLTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSSegwayRMPPOLTest(IsaacROSBaseTest):
    """Test for Isaac ROS SegwayRMP Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_segway_rmp_pipeline(self) -> None:
        if self.skip_test:
            self.skipTest('No segway detected! Skipping test.')
        else:
            """Expect the pipeline to produce output data from SegwayRMP lidar."""
            self.generate_namespace_lookup(
                ['battery_state', 'imu', 'odom', 'ticks'])

            received_messages = {}
            battery_state_sub, imu_sub, odom_sub, ticks_sub = self.create_logging_subscribers(
                [('battery_state', BatteryState),
                 ('imu', Imu),
                 ('odom', Odometry),
                 ('ticks', EncoderTicks)], received_messages,
                accept_multiple_messages=True)
            output_topics = ['battery_state', 'imu',
                             'odom', 'ticks']

            try:
                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 20
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received a message on the output topic, break
                    # Check if all messages have been received on output topics
                    if all(len(received_messages[topic]) > 0 for topic in output_topics):
                        done = True

                self.assertTrue(
                    done, "Didn't receive all outputs!")

            finally:
                pass
                self.assertTrue(self.node.destroy_subscription(battery_state_sub))
                self.assertTrue(self.node.destroy_subscription(imu_sub))
                self.assertTrue(self.node.destroy_subscription(odom_sub))
                self.assertTrue(self.node.destroy_subscription(ticks_sub))
