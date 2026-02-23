import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav  = get_package_share_directory('mecanum_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_mode    = LaunchConfiguration('slam_mode',    default='true')   # false → AMCL localization
    map_yaml     = LaunchConfiguration('map_yaml',     default='')

    nav2_params  = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params  = os.path.join(pkg_nav, 'config', 'slam_params.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
        condition=IfCondition(slam_mode)
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_mode',    default_value='true',
                              description='true=SLAM, false=AMCL localization'),
        DeclareLaunchArgument('map_yaml',     default_value='',
                              description='Path to map yaml (AMCL mode only)'),

        slam_launch,
        nav2_launch,
    ])

