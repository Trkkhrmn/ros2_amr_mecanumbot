# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    waypoints_yaml = LaunchConfiguration(
        'waypoints_yaml',
        default=os.path.expanduser('~/patrol_waypoints.yaml'),
    )
    use_simple_patrol = LaunchConfiguration('use_simple_patrol', default='true')

    # QR sadece Nav2 devriyede (patrol_manager) gerekir; basit devriyede yok
    qr_reader = Node(
        package='mecanum_task_manager',
        executable='qr_reader.py',
        name='qr_reader',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(use_simple_patrol),
    )

    # Simple patrol: odom + cmd_vel only, no Nav2
    simple_patrol = Node(
        package='mecanum_task_manager',
        executable='simple_patrol.py',
        name='simple_patrol',
        parameters=[{
            'waypoints_yaml': waypoints_yaml,
            'waypoint_tolerance': 0.4,
            'max_linear': 0.4,
            'max_angular': 0.6,
        }],
        output='screen',
        condition=IfCondition(use_simple_patrol),
    )

    # Patrol with Nav2 (map + planner + controller)
    patrol_manager = Node(
        package='mecanum_task_manager',
        executable='patrol_manager.py',
        name='patrol_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_yaml': waypoints_yaml,
            'waypoint_tolerance': 0.3,
        }],
        output='screen',
        condition=UnlessCondition(use_simple_patrol),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'waypoints_yaml',
            default_value=os.path.expanduser('~/patrol_waypoints.yaml'),
            description='YAML file with patrol_waypoints list',
        ),
        DeclareLaunchArgument(
            'use_simple_patrol',
            default_value='true',
            description='true = waypoint patrol with odom only (no Nav2); false = use Nav2',
        ),
        qr_reader,
        simple_patrol,
        patrol_manager,
    ])
