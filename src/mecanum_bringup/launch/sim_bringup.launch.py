import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim     = get_package_share_directory('mecanum_simulation')
    pkg_nav     = get_package_share_directory('mecanum_navigation')
    pkg_control = get_package_share_directory('mecanum_control')
    pkg_task    = get_package_share_directory('mecanum_task_manager')

    # warehouse devriye waypoint'leri (varsayılan)
    default_waypoints = os.path.join(pkg_task, 'config', 'warehouse_patrol_waypoints.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='warehouse')
    launch_navigation = LaunchConfiguration('launch_navigation', default='false')
    launch_control = LaunchConfiguration('launch_control', default='true')
    launch_task_manager = LaunchConfiguration('launch_task_manager', default='false')
    use_simple_patrol = LaunchConfiguration('use_simple_patrol', default='true')
    waypoints_yaml = LaunchConfiguration('waypoints_yaml', default=default_waypoints)

    # 1) Gazebo + robot spawn. world:= minimal | empty | warehouse
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'world': world}.items()
    )

    # 2) Navigation (SLAM + Nav2) — nav2_bringup ve slam_toolbox gerekir
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_navigation)
    )

    # 3) Control (lift + mecanum kinematics)
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'control.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_control)
    )

    # 4) Task Manager: use_simple_patrol=true → sadece odom ile dolaşım (Nav2 yok)
    task_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_task, 'launch', 'task_manager.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'waypoints_yaml': waypoints_yaml,
            'use_simple_patrol': use_simple_patrol,
        }.items(),
        condition=IfCondition(launch_task_manager)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            description='World: warehouse (varsayılan), minimal, empty'
        ),
        DeclareLaunchArgument(
            'launch_navigation',
            default_value='false',
            description='Launch Nav2 + SLAM (requires ros-humble-nav2-bringup, ros-humble-slam-toolbox)'
        ),
        DeclareLaunchArgument(
            'launch_control',
            default_value='true',
            description='Launch control nodes (lift, mecanum kinematics)'
        ),
        DeclareLaunchArgument(
            'launch_task_manager',
            default_value='false',
            description='Launch task manager and QR reader (requires Nav2)'
        ),
        DeclareLaunchArgument(
            'waypoints_yaml',
            default_value=default_waypoints,
            description='Patrol waypoints YAML (varsayılan: warehouse_patrol_waypoints.yaml)'
        ),
        DeclareLaunchArgument(
            'use_simple_patrol',
            default_value='true',
            description='true = odom ile waypoint dolaşımı (önerilen); false = Nav2 ile'
        ),
        simulation,
        navigation,
        control,
        task_manager,
    ])

