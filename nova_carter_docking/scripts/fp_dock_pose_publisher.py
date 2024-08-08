#!/usr/bin/env python3

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

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import math


class FPDockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'detected_dock_pose',
            1)
        self.subscription_ = self.create_subscription(
            Detection3DArray,
            '/tracking/output',
            self.detection_callback,
            1)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.saved_pose = None

    def not_within_range(self, x1, y1, range_limit):
        """ Calculate the Euclidean distance between the two points """
        distance = math.sqrt(x1 ** 2 + y1 ** 2)
        return distance > range_limit

    def detection_callback(self, msg):
        """ Publish the pose detected by Foundation Pose and save it as the most recent saved pose. """
        message = PoseStamped()
        message.header = msg.header
        bbox = msg.detections[0].bbox
        message.pose.position.x = bbox.center.position.x
        message.pose.position.y = bbox.center.position.y
        message.pose.position.z = bbox.center.position.z
        message.pose.orientation.w = bbox.center.orientation.w
        message.pose.orientation.x = bbox.center.orientation.x
        message.pose.orientation.y = bbox.center.orientation.y
        message.pose.orientation.z = bbox.center.orientation.z

        if self.not_within_range(message.pose.position.x, message.pose.position.z, 0.1):
            self.saved_pose = message

    def timer_callback(self):
        """ Publish the latest saved pose from Foundation Pose. """
        if self.saved_pose is None:
            return
        self.publisher_.publish(self.saved_pose)


def main(args=None):
    """ Execute the FPDockPosePublisher. """
    rclpy.init(args=args)
    node = FPDockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.disconnect()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
