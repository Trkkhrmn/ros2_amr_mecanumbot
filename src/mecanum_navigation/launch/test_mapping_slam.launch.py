"""
TEST 3 — FAZ 1: SLAM ile Haritalama
=====================================
Çalıştırma:
  ros2 launch mecanum_navigation test_mapping_slam.launch.py

Bu launch dosyası:
  1. Gazebo'yu test_mapping.world ile başlatır
  2. Robotu orijine spawn eder
  3. SLAM Toolbox'ı başlatır
  4. RViz'i açar (harita ve robot görünür)

Sonrasında:
  - Robotu teleop ile gezdirerek haritayı tamamla
  - Harita yeterince dolunca FAZ 2'ye geç (map_saver)

Haritayı kaydetmek için ayrı terminalde:
  ros2 run nav2_map_server map_saver_cli \\
    -f ~/mecanum_amr/src/mecanum_navigation/maps/warehouse_map \\
    --ros-args -p map_subscribe_transient_local:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav         = get_package_share_directory('mecanum_navigation')
    pkg_description = get_package_share_directory('mecanum_description')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')
    pkg_slam        = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file   = os.path.join(pkg_nav, 'worlds', 'test_mapping.world')
    urdf_file    = os.path.join(pkg_description, 'urdf', 'mecanum_amr.urdf.xacro')
    slam_params  = os.path.join(pkg_nav, 'config', 'slam_params.yaml')
    rviz_config  = os.path.join(pkg_nav, 'config', 'mapping.rviz')

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

    # Robot spawn — Gazebo ve robot_description hazır olsun diye 5 saniye gecikmeli
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

    # SLAM Toolbox — online async mod (robot spawn'dan sonra başlasın)
    slam = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params,
                }.items()
            ),
        ],
    )

    # RViz (SLAM ve robot hazır olsun diye biraz gecikmeli)
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
        set_model_path,
        gazebo,
        rsp,
        spawn,
        slam,
        rviz,
    ])
