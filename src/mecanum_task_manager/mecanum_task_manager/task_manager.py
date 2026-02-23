# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time   = LaunchConfiguration('use_sim_time',   default='true')
    waypoints_yaml = LaunchConfiguration('waypoints_yaml',
                        default=os.path.expanduser('~/patrol_waypoints.yaml'))

    qr_reader = Node(
        package='mecanum_task_manager',
        executable='qr_reader.py',
        name='qr_reader',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    patrol_manager = Node(
        package='mecanum_task_manager',
        executable='patrol_manager.py',
        name='patrol_manager',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'waypoints_yaml':     waypoints_yaml,
            'waypoint_tolerance': 0.3,
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('waypoints_yaml',
            default_value=os.path.expanduser('~/patrol_waypoints.yaml')),
        qr_reader,
        patrol_manager,
    ])

