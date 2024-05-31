// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_SEGWAY_RMP__SEGWAY_RMP_NODE_HPP_
#define ISAAC_ROS_SEGWAY_RMP__SEGWAY_RMP_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace segway_rmp
{

class SegwayRMPNode : public nitros::NitrosNode
{
public:
  explicit SegwayRMPNode(const rclcpp::NodeOptions &);

  ~SegwayRMPNode();

  SegwayRMPNode(const SegwayRMPNode &) = delete;

  SegwayRMPNode & operator=(const SegwayRMPNode &) = delete;

  // Callback to publish camera extrinsics to ROS TF tree
  void OdometryCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string robot_frame, const std::string odom_frame_name);

private:
  // Publisher for tf2.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};

  bool enable_odometry_tf_;
  std::string robot_frame_name_;
  std::string odom_frame_name_;
};

}  // namespace segway_rmp
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_SEGWAY_RMP__SEGWAY_RMP_NODE_HPP_
