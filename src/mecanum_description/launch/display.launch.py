import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('mecanum_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mecanum_amr.urdf.xacro')

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])

