"""
TEST 3 — FAZ 2: Lokalizasyon + Nav2 Navigasyon
================================================
Çalıştırma (harita kaydedildikten sonra):
  ros2 launch mecanum_navigation test_mapping_nav.launch.py \\
    map_yaml:=~/mecanum_amr/src/mecanum_navigation/maps/warehouse_map.yaml

Bu launch dosyası:
  1. Gazebo'yu aynı world ile başlatır
  2. Kaydedilen harita ile map_server'ı başlatır
  3. AMCL lokalizasyon başlatır
  4. Nav2 (planner + controller) başlatır
  5. RViz'i açar

Test adımları RViz'de:
  1. "2D Pose Estimate" ile robotun başlangıç pozisyonunu işaretle
  2. Robotu birkaç saniye gezdirerek AMCL'nin lokalize olmasını sağla
  3. "Nav2 Goal" ile mavi işaretçilere (goal_marker_1/2/3) hedef ver
  4. Robotun engelleri aşarak hedefe gittiğini doğrula

Beklenen:
  ✓ AMCL particle cloud yakınsıyor
  ✓ Planner engelsiz yol buluyor
  ✓ Controller hedefe ±5cm hassasiyetle ulaşıyor
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav         = get_package_share_directory('mecanum_navigation')
    pkg_description = get_package_share_directory('mecanum_description')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')
    pkg_nav2        = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml     = LaunchConfiguration('map_yaml')
    world_file   = os.path.join(pkg_nav, 'worlds', 'test_mapping.world')
    urdf_file    = os.path.join(pkg_description, 'urdf', 'mecanum_amr.urdf.xacro')
    nav2_params  = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(pkg_nav, 'config', 'navigation.rviz')

    # AWS model path
    sim_models = os.path.join(
        get_package_share_directory('mecanum_simulation'), 'models'
    )
    existing_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=sim_models + (':' + existing_path if existing_path else '')
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    # Robot state publisher
    robot_description = Command(['xacro ', urdf_file])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # Robot spawn — Gazebo hazır olsun diye gecikmeli
    spawn = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'mecanum_amr',
                    '-x', '0.0', '-y', '0.0', '-z', '0.15',
                ],
                output='screen',
            ),
        ],
    )

    # Nav2 — map_server + AMCL + planner + controller
    nav2 = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml,
                    'params_file': nav2_params,
                    'slam': 'False',
                }.items()
            ),
        ],
    )

    # RViz
    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(
                pkg_nav, 'maps', 'warehouse_map.yaml'
            ),
            description='Kaydedilen harita YAML dosyasının tam yolu'
        ),
        set_model_path,
        gazebo,
        rsp,
        spawn,
        nav2,
        rviz,
    ])
