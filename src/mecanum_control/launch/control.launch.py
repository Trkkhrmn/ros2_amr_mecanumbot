# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # mecanum_kinematics is a library, not a node; only lift_controller runs
    lift_controller = Node(
        package='mecanum_control',
        executable='lift_controller.py',
        name='lift_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        lift_controller,
    ])
