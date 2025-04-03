# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from nav2_common.launch import RewrittenYaml


# Override parameters for carter in MEGA using static map costmap only
def set_override_parameters_for_mega(args: lu.ArgumentContainer) -> list[lut.Action]:
    local_costmap_plugins = ['static_map_layer', 'inflation_layer']
    global_costmap_plugins = ['static_map_layer', 'inflation_layer']

    print(f'Using local costmap plugins: {local_costmap_plugins}')
    print(f'Using global costmap plugins: {global_costmap_plugins}')
    actions: list[lut.Action] = []
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='plugins',
            value=local_costmap_plugins,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='global_frame',
            value='map',
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
            value='/chassis/odom',
        ))
    actions.append(
        lu.set_parameter(
            namespace='/velocity_smoother',
            parameter='odom_topic',
            value='/chassis/odom',
        ))
    actions.append(
        lu.set_parameter(
            namespace='/controller_server',
            parameter='odom_topic',
            value='/chassis/odom',
        ))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('rosbag', 'None', cli=True)
    args.add_arg('mode', 'mega', cli=True)
    args.add_arg('navigation_parameters_path',
                 lu.get_path('isaac_ros_mega_controller',
                             'params/nova_carter_navigation_mega.yaml'),
                 cli=True)
    args.add_arg('map_yaml_path', 'None', cli=True)
    args.add_arg('enable_navigation', True, cli=True)
    args.add_arg('enable_mission_client', False, cli=True)
    args.add_arg('enable_3d_lidar_costmap', False, cli=True)
    args.add_arg('enable_2d_lidar_costmap', True, cli=True)
    args.add_arg('enable_nvblox_costmap', True, cli=True)
    args.add_arg('enable_wheel_odometry', True, cli=True)
    args.add_arg('enable_3d_lidar_localization', True, cli=True)
    args.add_arg('enable_visual_localization', False, cli=True)
    args.add_arg('stereo_camera_configuration',
                 default='front_configuration',
                 choices=['no_cameras', 'front_configuration',
                          'front_left_right_configuration', 'front_driver_rectify',
                          'front_left_right_vslam_configuration'],
                 cli=True)
    args.add_arg('enabled_fisheye_cameras', '', cli=True)
    args.add_arg('use_foxglove_whitelist', True, cli=True)
    args.add_arg('init_pose_x', '', cli=True)
    args.add_arg('init_pose_y', '', cli=True)
    args.add_arg('init_pose_yaw', '', cli=True)
    args.add_arg('enable_controller', False, cli=True)
    args.add_arg('enable_metrics', False, cli=True)
    args.add_arg('metrics_interval', 15.0, cli=True)
    args.add_arg('waypoints', 'None', cli=True)
    args.add_arg('type_negotiation_duration_s', lu.get_default_negotiation_time(), cli=True)

    enable_global_navigation = lu.is_valid(args.map_yaml_path)

    # map_only is true if nvblox and lidars are not enabled
    enable_lidar = lut.OrSubstitution(
        args.enable_3d_lidar_costmap, args.enable_2d_lidar_costmap)
    map_only = lut.AndSubstitution(
        lut.NotSubstitution(enable_lidar), lut.NotSubstitution(args.enable_nvblox_costmap))

    actions = args.get_launch_actions()
    actions.append(lut.SetParameter('type_negotiation_duration_s',
                   args.type_negotiation_duration_s))
    actions.append(
        lu.log_info(['Using type negotiation duration: ', args.type_negotiation_duration_s]))
    actions.append(lut.SetParameter('use_sim_time', True))

    # NOTE: If running in sim mode we do mapping in another frame to not clash with ground truth
    # coming out of Isaac Sim.
    # Set global frame to odom_vslam when:
    # - running local navigation in Isaac Sim (i.e. no global map passed)
    run_local_navigation_in_sim = lu.is_false(enable_global_navigation)
    global_frame = lu.if_else_substitution(run_local_navigation_in_sim, 'odom_vslam', 'odom')

    # NOTE: If running in sim mode we are setting the reliability policy of vslam to reliable
    # as we were experiencing issues with the best effort policy
    # (i.e. not able to synchronize input messages due to dropped images).
    vslam_image_qos = 'DEFAULT'

    # Add the navigation.
    actions.append(
        lu.include(
            'nova_carter_bringup',
            'launch/include/navigation_include.launch.py',
            launch_arguments={
                'global_frame': global_frame,
                'vslam_image_qos': vslam_image_qos,
                'init_pose_x': args.init_pose_x,
                'init_pose_y': args.init_pose_y,
                'init_pose_yaw': args.init_pose_yaw,
                'set_init_pose': True,
            },
            condition=lut.IfCondition(lut.NotSubstitution(map_only)),
        ))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/tools/visualization.launch.py',
            launch_arguments={'use_foxglove_whitelist': args.use_foxglove_whitelist},
            condition=lut.IfCondition(lut.NotSubstitution(map_only)),
        ))

    controller = lut.Node(
        package='isaac_ros_mega_controller',
        executable='isaac_ros_mega_controller',
        name='isaac_ros_mega_controller',
        output='screen',
        parameters=[{'waypoints': args.waypoints}],
        condition=lut.IfCondition(args.enable_controller),
    )
    actions.append(controller)

    # We use a negative niceness to increase the priority.
    actions.append(lu.component_container('nova_container', prefix='nice -n -10'))
    # A separate container is needed for Nav2 because ROS2 has a race condition for actions when
    # using non-isolated containers.
    # Nav container has to be added last, else Nav2 parameters are not picked up properly.
    actions.append(lu.component_container('navigation_container', container_type='isolated'))

    # If only static map is used, override parameter file to use static map
    # for local and global costmap, launch nav2 navigation and map server,
    # pulish map->odom tf.
    navigation_parameters_path = lu.if_else_substitution(
        map_only,
        RewrittenYaml(source_file=args.navigation_parameters_path,
                      root_key='',
                      param_rewrites={'yaml_filename': args.map_yaml_path},
                      convert_types=True),
        args.navigation_parameters_path
    )

    actions.append(lut.SetParametersFromFile(navigation_parameters_path))
    actions.append(
        args.add_opaque_function(
            set_override_parameters_for_mega,
            condition=lut.IfCondition(map_only),
        ))

    actions.append(
        lu.include(
            'nav2_bringup',
            'launch/navigation_launch.py',
            launch_arguments={
                'use_sim_time': True,
                'params_file': navigation_parameters_path
            },
            condition=lut.IfCondition(map_only),
        ))

    actions.append(
        lu.include(
            'isaac_ros_mega_node_monitor',
            'launch/isaac_ros_mega_node_monitor.launch.py',
            launch_arguments={
                'enable_metrics': args.enable_metrics,
                'metrics_interval': args.metrics_interval,
                'enable_3d_lidar_costmap': args.enable_3d_lidar_costmap
            },
            condition=lut.IfCondition(args.enable_navigation)
        ))

    actions.append(
        lut.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            arguments=[args.init_pose_x, args.init_pose_y,
                       '0', args.init_pose_yaw, '0', '0', 'map', 'odom'],
            output='screen',
            condition=lut.IfCondition(map_only),
        )
    )

    actions.append(
        lu.include(
            'nova_carter_description',
            'launch/nova_carter_description.launch.py',
            condition=lut.IfCondition(map_only),
        ))

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    actions.append(
        lut.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[{'yaml_filename': args.map_yaml_path}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=remappings,
            condition=lut.IfCondition(map_only),
        ))

    actions.append(
        lut.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'autostart': True}, {'node_names': ['map_server']}],
            condition=lut.IfCondition(map_only),
        ))

    actions.append(
        lu.include(
            'isaac_ros_vda5050_nav2_client_bringup',
            'launch/isaac_ros_vda5050_client.launch.py',
            launch_arguments={
                'docking_server_enabled': False,
            },
            condition=lut.IfCondition(
                lut.AndSubstitution(args.enable_mission_client, map_only)
            ),
        ))

    return lut.LaunchDescription(actions)
