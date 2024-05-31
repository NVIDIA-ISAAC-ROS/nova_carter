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
from nav2_common.launch import RewrittenYaml


def set_override_parameters(args: lu.ArgumentContainer) -> list[Action]:
    local_costmap_plugins = []
    global_costmap_plugins = []

    # Supported configurations
    # - Real Carter
    #   - A mixture of nvblox, 2D, and 3D LiDAR
    # - Isaac Sim Carter
    #   - nvblox
    if args.mode == 'simulation':
        print('In simulation only nvblox costmap supported.')
        print('Enabling nvblox costmap.')
        local_costmap_plugins.append('nvblox_layer')
        global_costmap_plugins.append('nvblox_layer')
        odometry_topic = '/visual_slam/tracking/odometry'
    else:
        enable_global_navigation = lu.is_valid(args.map_yaml_path)
        if enable_global_navigation:
            print('Enabling global navigation.')
            global_costmap_plugins.append('static_map_layer')

        if args.enable_2d_lidar_costmap:
            for name in ['front_2d_lidar', 'back_2d_lidar']:
                print(f'Enabling 2d lidar costmap for {name}.')
                local_costmap_plugins.append(f'{name}_layer')

        if args.enable_3d_lidar_costmap:
            print('Enabling 3d lidar costmap.')
            local_costmap_plugins.append('3d_lidar_layer')

        if args.enable_nvblox_costmap:
            print('Enabling nvblox costmap.')
            local_costmap_plugins.append('nvblox_layer')

        if args.enable_wheel_odometry:
            odometry_topic = '/chassis/odom'
        else:
            odometry_topic = '/visual_slam/tracking/odometry'

    local_costmap_plugins += ['inflation_layer']
    global_costmap_plugins += ['inflation_layer']

    print(f'Using local costmap plugins: {local_costmap_plugins}')
    print(f'Using global costmap plugins: {global_costmap_plugins}')
    actions: list[Action] = []
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='plugins',
            value=local_costmap_plugins,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='plugins',
            value=global_costmap_plugins,
        ))

    actions.append(
        lu.set_parameter(
            namespace='/bt_navigator',
            parameter='odom_topic',
            value=odometry_topic,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/velocity_smoother',
            parameter='odom_topic',
            value=odometry_topic,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/controller_server',
            parameter='odom_topic',
            value=odometry_topic,
        ))

    if args.enable_3d_lidar_localization:
        scan_topic = '/front_3d_lidar/scan'
    else:
        scan_topic = '/front_2d_lidar/scan'

    actions.append(lu.set_parameter(
        namespace='/amcl',
        parameter='scan_topic',
        value=scan_topic,
    ))

    return actions


def rewrite_navigation_parameters_for_sim(navigation_parameters_path: LaunchConfiguration,
                                          global_frame: LaunchConfiguration) -> RewrittenYaml:
    # For simulation we configure navigation to occur in the VSLAM frame to not clash
    # with the Isaac Sim published 'odom' frame.
    param_substitutions = {'global_frame': global_frame, 'nav2_costmap_global_frame': global_frame}
    return RewrittenYaml(source_file=navigation_parameters_path,
                         root_key='',
                         param_rewrites=param_substitutions,
                         convert_types=True)


def check_args(args: lu.ArgumentContainer):
    # Some of are arguments are not supported when running in Isaac Sim.
    # This function checks the passed arguments.
    if args.mode == 'simulation':
        if args.map_yaml_path != None:
            print(
                f"Map passed to navigation in Isaac Sim: {args.map_yaml_path}. This feature is not "
                "supported yet. Map ignored, performing local navigation.")
        if (args.enable_2d_lidar_costmap == True):
            print("Navigation with 2D LiDAR not supported yet in sim. 2D LiDARs will not be used.")
        if (args.enable_3d_lidar_costmap == True):
            print("Navigation with 3D LiDAR not supported yet in sim. 3D LiDARs will not be used.")
        assert (
            not args.global_frame == 'odom'
        ), "Tried to use odom as the global frame. This will cause collisions between the Isaac Sim "
        "supplied GT pose and cuVSLAM. Please use another global frame name."
        assert (args.stereo_camera_configuration == None
                or args.stereo_camera_configuration == 'front_configuration'
                ), "Only stereo_camera_configuration:=front_configuration supported in Isaac Sim."

    # Wheel odometry or stereo cameras have to be enabled.
    # Otherwise the robot will not have any odometry source
    assert (args.enable_wheel_odometry == True or args.stereo_camera_configuration
            != None), "Stereo camera(s) or wheel odometry has to be enabled."


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('navigation_parameters_path')
    args.add_arg('stereo_camera_configuration', None)
    args.add_arg('mode')
    args.add_arg('global_frame', 'odom')
    args.add_arg('map_yaml_path')
    args.add_arg('enable_navigation')
    args.add_arg('enable_mission_client')
    args.add_arg('enable_3d_lidar_costmap')
    args.add_arg('enable_2d_lidar_costmap')
    args.add_arg('enable_nvblox_costmap')
    args.add_arg('enable_wheel_odometry')
    args.add_arg('enable_3d_lidar_localization')

    args.add_opaque_function(check_args)

    is_sim = lu.is_equal(args.mode, 'simulation')

    enable_global_navigation = AndSubstitution(lu.is_valid(args.map_yaml_path), lu.is_not(is_sim))
    enable_3d_lidar = OrSubstitution(args.enable_3d_lidar_costmap,
                                     args.enable_3d_lidar_localization)
    enable_2d_lidars = OrSubstitution(args.enable_2d_lidar_costmap,
                                      NotSubstitution(args.enable_3d_lidar_localization))
    enabled_2d_lidars = lu.if_else_substitution(enable_2d_lidars, 'front_2d_lidar,back_2d_lidar',
                                                '')
    disable_cuvslam = AndSubstitution(args.enable_wheel_odometry, lu.is_not(is_sim))

    actions = args.get_launch_actions()

    # When running in Isaac Sim we rewrite the parameter file to adjust the frame names.
    actions.append(lu.assert_path_exists(args.navigation_parameters_path))
    navigation_parameters_path = lu.if_else_substitution(
        is_sim,
        rewrite_navigation_parameters_for_sim(args.navigation_parameters_path, args.global_frame),
        args.navigation_parameters_path)

    actions.append(SetParametersFromFile(navigation_parameters_path))
    # We have to add the opaque function after having set the default
    # parameters. This will also add the opaque function to the actions in
    # 'args.get_launch_actions' which we already added above, but we explicitly
    # want to only add it now.
    actions.append(args.add_opaque_function(set_override_parameters))

    # Add perceptor.
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/include/perceptor_include.launch.py',
            launch_arguments={
                'enable_3d_lidar': enable_3d_lidar,
                'enabled_2d_lidars': enabled_2d_lidars,
                'global_frame': args.global_frame,
                'invert_odom_to_base_tf': is_sim,
                'disable_cuvslam': disable_cuvslam,
            },
        ))

    # Add lidar localization.
    actions.append(
        lu.include(
            'nova_carter_navigation',
            'launch/lidar_localization.launch.py',
            launch_arguments={
                'map_yaml_path': args.map_yaml_path,
                'navigation_parameters_path': navigation_parameters_path,
                'enable_3d_lidar_localization': args.enable_3d_lidar_localization,
            },
            condition=IfCondition(enable_global_navigation),
        ))
    actions.append(
        lu.static_transform(
            'map',
            args.global_frame,
            condition=UnlessCondition(enable_global_navigation),
        ))

    # Add navigation.
    actions.append(
        lu.include(
            'nova_carter_navigation',
            'launch/navigation.launch.py',
            launch_arguments={
                'navigation_parameters_path': navigation_parameters_path,
                'enable_mission_client': args.enable_mission_client,
            },
            condition=IfCondition(args.enable_navigation),
        ))

    return LaunchDescription(actions)
