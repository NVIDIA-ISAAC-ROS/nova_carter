// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "isaac_ros_segway_rmp/segway_rmp_node.hpp"
#include "isaac_ros_nitros_twist_type/nitros_twist.hpp"
#include "isaac_ros_nitros_odometry_type/nitros_odometry.hpp"
#include "isaac_ros_nitros_encoder_ticks_type/nitros_encoder_ticks.hpp"
#include "isaac_ros_nitros_battery_state_type/nitros_battery_state.hpp"
#include "isaac_ros_nitros_imu_type/nitros_imu.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/timestamp.hpp"
#include "extensions/messages/composite_message.hpp"
#include "./comm_ctrl_navigation.h"
#pragma GCC diagnostic pop


namespace nvidia
{
namespace isaac_ros
{
namespace segway_rmp
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

#define INPUT_COMPONENT_KEY_CMD_VEL                "segway_driver/rx_command"
#define INPUT_DEFAULT_TENSOR_FORMAT_CMD_VEL        "nitros_twist"
#define INPUT_TOPIC_NAME_CMD_VEL                   "cmd_vel"

#define OUTPUT_COMPONENT_KEY_ODOMETRY              "sink_odometry/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_ODOMETRY      "nitros_odometry"
#define OUTPUT_TOPIC_NAME_ODOMETRY                 "odom"

#define OUTPUT_COMPONENT_KEY_ENCODER_TICKS         "sink_encoder_ticks/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_ENCODER_TICKS "nitros_encoder_ticks"
#define OUTPUT_TOPIC_NAME_ENCODER_TICKS            "ticks"

#define OUTPUT_COMPONENT_KEY_BATTERY_STATE         "sink_battery_state/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_BATTERY_STATE "nitros_battery_state"
#define OUTPUT_TOPIC_NAME_BATTERY_STATE            "battery_state"

#define OUTPUT_COMPONENT_KEY_IMU                   "sink_imu/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_IMU           "nitros_imu"
#define OUTPUT_TOPIC_NAME_IMU                      "imu"

constexpr char APP_YAML_FILENAME[] = "config/segway_rmp_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_segway_rmp";
constexpr int kPositionXIndx = 0;
constexpr int kPositionYIndx = 1;
constexpr int kHeadingIndx = 2;

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so"},  // rangescan info
  {"isaac_ros_gxf", "gxf/lib/libgxf_segway.so"}
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {
  "config/extension_specs.yaml"
};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule_segway_rmp.yaml"
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {INPUT_COMPONENT_KEY_CMD_VEL,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = INPUT_DEFAULT_TENSOR_FORMAT_CMD_VEL,
      .topic_name = INPUT_TOPIC_NAME_CMD_VEL,
    }
  },
  {OUTPUT_COMPONENT_KEY_ODOMETRY,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_ODOMETRY,
      .topic_name = OUTPUT_TOPIC_NAME_ODOMETRY,
    }
  },
  {OUTPUT_COMPONENT_KEY_ENCODER_TICKS,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_ENCODER_TICKS,
      .topic_name = OUTPUT_TOPIC_NAME_ENCODER_TICKS,
    }
  },
  {OUTPUT_COMPONENT_KEY_BATTERY_STATE,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_BATTERY_STATE,
      .topic_name = OUTPUT_TOPIC_NAME_BATTERY_STATE,
    }
  },
  {OUTPUT_COMPONENT_KEY_IMU,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_IMU,
      .topic_name = OUTPUT_TOPIC_NAME_IMU,
    }
  }
};
#pragma GCC diagnostic pop

SegwayRMPNode::SegwayRMPNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  RCLCPP_DEBUG(get_logger(), "[SegwayRMPNode] Constructor");
  robot_frame_name_ = declare_parameter<std::string>("robot_frame_name", "base_link");
  odom_frame_name_ = declare_parameter<std::string>("odom_frame_name", "odom");

  // Adding callback for left camerainfo
  config_map_[OUTPUT_COMPONENT_KEY_ODOMETRY].callback =
    std::bind(
    &SegwayRMPNode::OdometryCallback, this,
    std::placeholders::_1, std::placeholders::_2, robot_frame_name_,
    odom_frame_name_);

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosTwist>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosOdometry>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosEncoderTicks>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosBatteryState>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImu>();

  startNitrosNode();

  // Ensure segway cleanup runs on shutdown in case the process dies
  rclcpp::on_shutdown(
    [&]
    {
      set_enable_ctrl(0);
      exit_control_ctrl();
    });
}

void SegwayRMPNode::OdometryCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg,
  const std::string robot_frame, const std::string odom_frame_name)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = odom_frame_name;
  transform_stamped.child_frame_id = robot_frame;
  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);
  // Create composite msg and populate values
  auto maybe_composite_message = nvidia::isaac::GetCompositeMessage(msg_entity.value());
  if (!maybe_composite_message) {
    std::stringstream error_msg;
    error_msg <<
      "[SegwayRMPNode] Failed to get maybe_composite_message gxf message"
      " from message entity in OdometryCallback " <<
      GxfResultStr(maybe_composite_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("SegwayRMPNode"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto composite_message = maybe_composite_message.value();

  // Populate odometry data of the ROS msg from gxf msg
  transform_stamped.transform.translation.x = composite_message.view(0, kPositionXIndx);
  transform_stamped.transform.translation.y = composite_message.view(0, kPositionYIndx);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, composite_message.view(0, kHeadingIndx));
  transform_stamped.transform.rotation.x = quaternion.getX();
  transform_stamped.transform.rotation.y = quaternion.getY();
  transform_stamped.transform.rotation.z = quaternion.getZ();
  transform_stamped.transform.rotation.w = quaternion.getW();

  // Populate timestamp information back into ROS header
  auto input_timestamp = composite_message.timestamp;
  transform_stamped.header.stamp.sec = static_cast<int32_t>(
    composite_message.timestamp->acqtime / static_cast<uint64_t>(1e9));
  transform_stamped.header.stamp.nanosec = static_cast<uint32_t>(
    composite_message.timestamp->acqtime % static_cast<uint64_t>(1e9));

  tf_broadcaster_->sendTransform(transform_stamped);
}

SegwayRMPNode::~SegwayRMPNode() {}

}  // namespace segway_rmp
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::segway_rmp::SegwayRMPNode)
