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
from launch_ros.actions import SetRemap


def set_override_parameters(args: lu.ArgumentContainer) -> list[Action]:
    local_costmap_plugins = []
    global_costmap_plugins = []

    # Supported configurations
    # - Real Carter
    #   - A mixture of nvblox, 2D, and 3D LiDAR
    # - Isaac Sim Carter
    #   - nvblox
    # - MEGA
    #   - A mixture of nvblox, 2D, and 3D LiDAR
    enable_global_navigation = lu.is_valid(args.map_yaml_path)
    if enable_global_navigation:
        print('Enabling global navigation.')
        global_costmap_plugins.append('static_map_layer')

    if args.mode == 'simulation':
        print('Enabling nvblox costmap.')
        local_costmap_plugins.append('nvblox_layer')
        global_costmap_plugins.append('nvblox_layer')
        odometry_topic = '/visual_slam/tracking/odometry'
    else:
        if args.enable_2d_lidar_costmap:
            for name in ['front_2d_lidar', 'back_2d_lidar']:
                print(f'Enabling 2d lidar costmap for {name}.')
                local_costmap_plugins.append(f'{name}_layer')
                global_costmap_plugins.append(f'{name}_layer')

        if args.enable_3d_lidar_costmap:
            print('Enabling 3d lidar costmap.')
            local_costmap_plugins.append('3d_lidar_layer')
            global_costmap_plugins.append('3d_lidar_layer')

        if args.enable_nvblox_costmap:
            print('Enabling nvblox costmap.')
            local_costmap_plugins.append('nvblox_layer')
            global_costmap_plugins.append('nvblox_layer')

        # Disable wheel odome when visual localization is enabled.
        if args.enable_wheel_odometry and not args.enable_visual_localization:
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

    # configure lidar localization when visual localization is not enabled
    if not args.enable_visual_localization:
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

def rewrite_init_pose(
    navigation_parameters_path: LaunchConfiguration,
    x: LaunchConfiguration, 
    y: LaunchConfiguration, 
    yaw: LaunchConfiguration) -> RewrittenYaml:
    # If set_init_pose is True, rewrite the amcl config
    param_substitutions = {'set_initial_pose': 'True', 'x': x, 'y': y, 'yaw': yaw}
    return RewrittenYaml(source_file=navigation_parameters_path,
                         root_key='',
                         param_rewrites=param_substitutions,
                         convert_types=True)



def check_args(args: lu.ArgumentContainer):
    # Wheel odometry or stereo cameras have to be enabled.
    # Otherwise the robot will not have any odometry source
    assert (args.enable_wheel_odometry == True or args.stereo_camera_configuration
            != None), "Stereo camera(s) or wheel odometry has to be enabled."


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('navigation_parameters_path')
    args.add_arg('stereo_camera_configuration')
    args.add_arg('mode')
    args.add_arg('global_frame', 'odom')
    args.add_arg('vslam_image_qos', 'SENSOR_DATA')
    args.add_arg('map_yaml_path')
    args.add_arg('enable_navigation')
    args.add_arg('enable_mission_client')
    args.add_arg('enable_docking', False)
    args.add_arg('enable_3d_lidar_costmap')
    args.add_arg('enable_2d_lidar_costmap')
    args.add_arg('enable_nvblox_costmap')
    args.add_arg('enable_wheel_odometry')
    args.add_arg('enable_3d_lidar_localization')
    args.add_arg('enable_visual_localization')
    args.add_arg('set_init_pose', False)
    args.add_arg('init_pose_x', '0.0')
    args.add_arg('init_pose_y', '0.0')
    args.add_arg('init_pose_yaw', '0.0')

    args.add_opaque_function(check_args)

    is_sim = OrSubstitution(
        lu.is_equal(args.mode, 'simulation'),
        lu.is_equal(args.mode, 'mega')
    )

    enable_global_navigation = lu.is_valid(args.map_yaml_path)

    # We disable lidar localization if a user requests visual localization
    enable_lidar_localization = lu.AndSubstitution(
        enable_global_navigation, lu.NotSubstitution(args.enable_visual_localization))

    # We disable wheel_odometry if a user requests visual localization
    enable_wheel_odometry = lu.AndSubstitution(args.enable_wheel_odometry,
                                               lu.NotSubstitution(args.enable_visual_localization))

    # Enable 3d lidar localization if lidar localization is enabled and user requests 3d ldiar
    enable_3d_lidar_localization = lu.AndSubstitution(
        enable_lidar_localization, args.enable_3d_lidar_localization)

    # Enable 2d lidar localization if lidar localization is enabled and users requests 2d lidar
    enable_2d_lidar_localization = lu.AndSubstitution(
        enable_lidar_localization, lu.NotSubstitution(args.enable_3d_lidar_localization))

    enable_3d_lidar = OrSubstitution(args.enable_3d_lidar_costmap,
                                     enable_3d_lidar_localization)
    enable_2d_lidars = OrSubstitution(args.enable_2d_lidar_costmap,
                                      enable_2d_lidar_localization)
    enabled_2d_lidars = lu.if_else_substitution(enable_2d_lidars, 'front_2d_lidar,back_2d_lidar',
                                                '')
    # Only disable cuvslam when:
    # - doing global navigation in Isaac Sim
    # - running wheel odom on real robot
    # TODO: enable cuvslam for global navigation in Sim when global localization
    # is supported.
    disable_cuvslam = OrSubstitution(
        AndSubstitution(enable_wheel_odometry, lu.is_false(is_sim)),
        AndSubstitution(enable_global_navigation, lu.is_true(is_sim))
    )

    actions = args.get_launch_actions()

    # When running in Isaac Sim we rewrite the parameter file to adjust the frame names.
    actions.append(lu.assert_path_exists(args.navigation_parameters_path))
    # Rewrite nav parameters when:
    # - running local navigation in Isaac Sim (i.e. no global map passed)
    run_local_navigation_in_sim = AndSubstitution(
        lu.is_false(enable_global_navigation), lu.is_true(is_sim))
    navigation_parameters_path = lu.if_else_substitution(
        run_local_navigation_in_sim,
        rewrite_navigation_parameters_for_sim(args.navigation_parameters_path, args.global_frame),
        args.navigation_parameters_path)

    navigation_parameters_path = lu.if_else_substitution(
        args.set_init_pose,
        rewrite_init_pose(navigation_parameters_path, args.init_pose_x, args.init_pose_y, args.init_pose_yaw),
        navigation_parameters_path)

    # Launch a map server only when map is provided and lidar_localization.launch.py is not included
    launch_map_server = AndSubstitution(lu.NotSubstitution(
        enable_lidar_localization), lu.is_valid(args.map_yaml_path))

    # The yaml_filename in navigation_parameters_path somehow overrides the one in occupancy_map_server.launch.py.
    # We change the parameter in navigation_parameters_path to the input map file.
    navigation_parameters_path = lu.if_else_substitution(
        launch_map_server,
        RewrittenYaml(source_file=navigation_parameters_path,
                      root_key='',
                      param_rewrites={'yaml_filename': args.map_yaml_path},
                      convert_types=True),
        navigation_parameters_path
    )

    actions.append(lu.log_info(['Load navigation parameters from: ', navigation_parameters_path]))
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
                'vslam_odom_frame': args.global_frame,
                # When publish_map_to_odom_tf is true, the AMCL localization pose somehow becomes ineffective.
                # Do not publish map tf in vslam when lidar localization is enabled.
                'vslam_publish_map_to_odom_tf': lu.is_false(enable_lidar_localization),
                'nvblox_global_frame': args.global_frame,
                'vslam_image_qos': args.vslam_image_qos,
                'invert_odom_to_base_tf': is_sim,
                'disable_cuvslam': disable_cuvslam,
                'disable_nvblox': lu.is_false(args.enable_nvblox_costmap),
                'is_sim': is_sim,
                'disable_vgl': lu.is_false(args.enable_visual_localization),
                'occupancy_map_yaml_file': lu.if_else_substitution(launch_map_server, args.map_yaml_path, ''),
                'enable_wheel_odometry': enable_wheel_odometry,
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
                'use_sim_time': is_sim,
                'set_init_pose': args.set_init_pose,
            },
            condition=IfCondition(enable_lidar_localization),
        ))
    # Publish an identity transformation from map to global_frame if no global localization is provided.
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
                'enable_docking': args.enable_docking,
            },
            condition=IfCondition(args.enable_navigation),
        ))

    return LaunchDescription(actions)
