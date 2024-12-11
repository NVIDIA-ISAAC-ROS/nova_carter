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
import isaac_ros_perceptor_python_utils.launch_utils as pu


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()

    # String specifying the stereo camera configuration.
    args.add_arg('stereo_camera_configuration')
    args.add_arg('nvblox_global_frame', 'odom')
    args.add_arg('vslam_map_frame', 'map')
    args.add_arg('vslam_odom_frame', 'odom')
    args.add_arg('vslam_image_qos')
    args.add_arg('disable_cuvslam', False)
    args.add_arg('disable_nvblox', False)
    args.add_arg('disable_vgl', True)
    args.add_arg('nvblox_param_filename', 'params/nvblox_perceptor.yaml')
    args.add_arg('nvblox_after_shutdown_map_save_path', '')
    args.add_arg('occupancy_map_yaml_file', '')
    args.add_arg('is_sim', False)
    actions = args.get_launch_actions()

    perceptor_configuration, loggings = pu.load_perceptor_configuration(
        args.stereo_camera_configuration, args.disable_cuvslam, args.disable_nvblox, args.disable_vgl)

    actions.extend(loggings)

    enabled_stereo_cameras_drivers = lu.get_keys_with_substring_in_value(
        perceptor_configuration, 'driver')
    enable_people_segmentation = lu.dict_values_contain_substring(perceptor_configuration,
                                                                  'nvblox_people')
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/include/hardware_abstraction_layer_include.launch.py',
            launch_arguments={
                'enabled_stereo_cameras': enabled_stereo_cameras_drivers,
                'enable_people_segmentation': enable_people_segmentation
            },
        ))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/perceptor_general.launch.py',
            launch_arguments={
                'perceptor_configuration': perceptor_configuration,
                'nvblox_global_frame': args.nvblox_global_frame,
                'vslam_odom_frame': args.vslam_odom_frame,
                'vslam_map_frame': args.vslam_map_frame,
                'nvblox_param_filename': args.nvblox_param_filename,
                'nvblox_after_shutdown_map_save_path': args.nvblox_after_shutdown_map_save_path,
                'vslam_image_qos': args.vslam_image_qos,
                'is_sim': args.is_sim,
                'occupancy_map_yaml_file': args.occupancy_map_yaml_file,
            },
        ))

    return LaunchDescription(actions)
