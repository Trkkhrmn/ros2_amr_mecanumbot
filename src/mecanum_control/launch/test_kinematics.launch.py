# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

"""
Mecanum kinematics + keyboard control test.
  ros2 launch mecanum_control test_kinematics.launch.py

With teleop_twist_keyboard: i=forward, ,=back, j=rotate left, l=rotate right,
u/o=diagonal. Mecanum lateral (a/d) may need a custom teleop node.
Expected: forward/back = all 4 wheels same speed; lateral = diagonal pairs opposite;
rotate = left wheels back, right forward; watchdog stops after 500ms without cmd.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory('mecanum_control')
    pkg_description = get_package_share_directory('mecanum_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(pkg_control, 'worlds', 'test_kinematics.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mecanum_amr.urdf.xacro')

    # Gazebo — minimal world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'verbose': 'false',
            'pause':   'false',
        }.items()
    )

    # Robot description
    robot_description = Command(['xacro ', urdf_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Robot spawn — başlangıç: orijin
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mecanum_amr',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen'
    )

    # C++ Mecanum Drive Node
    drive_node = Node(
        package='mecanum_control',
        executable='mecanum_drive_node',
        name='mecanum_drive_node',
        parameters=[{
            'use_sim_time':    use_sim_time,
            'wheel_radius':    0.075,
            'lx':              0.25,
            'ly':              0.35,
            'max_wheel_speed': 15.0,
        }],
        output='screen',
        # Remap so we can watch wheel speeds in test
        remappings=[
            ('wheel_speeds', '/wheel_speeds_debug'),
        ]
    )

    # Wheel speed monitor for debugging
    wheel_monitor = Node(
        package='mecanum_control',
        executable='wheel_speed_monitor.py',
        name='wheel_speed_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[('wheel_speeds', '/wheel_speeds_debug')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        rsp,
        spawn,
        drive_node,
        wheel_monitor,
    ])
