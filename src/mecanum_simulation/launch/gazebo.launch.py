import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _get_aws_warehouse_paths():
    """AWS small warehouse: önce AWS_WAREHOUSE_ROOT env, yoksa kurulu paket."""
    aws_root = os.environ.get('AWS_WAREHOUSE_ROOT', '').strip()
    if aws_root and os.path.isdir(aws_root):
        models_path = os.path.join(aws_root, 'models')
        world_path = os.path.join(aws_root, 'worlds', 'small_warehouse', 'small_warehouse.world')
        return models_path if os.path.isdir(models_path) else None, world_path if os.path.isfile(world_path) else None
    try:
        pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')
        models_path = os.path.join(pkg, 'models')
        world_path = os.path.join(pkg, 'worlds', 'small_warehouse', 'small_warehouse.world')
        return models_path if os.path.isdir(models_path) else None, world_path if os.path.isfile(world_path) else None
    except Exception:
        return None, None


def generate_launch_description():
    pkg_sim         = get_package_share_directory('mecanum_simulation')
    pkg_description = get_package_share_directory('mecanum_description')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name   = LaunchConfiguration('world', default='warehouse')
    urdf_file    = os.path.join(pkg_description, 'urdf', 'mecanum_amr.urdf.xacro')

    # Kendi QR modellerimiz her zaman ilk sırada
    sim_models_path = os.path.join(pkg_sim, 'models')
    aws_models_path, aws_world_path = _get_aws_warehouse_paths()
    existing_path   = os.environ.get('GAZEBO_MODEL_PATH', '')
    parts = [sim_models_path]
    if aws_models_path:
        parts.append(aws_models_path)
    if existing_path:
        parts.append(existing_path)
    full_model_path = ':'.join(parts)

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=full_model_path
    )

    # World dosyası: small_warehouse ise AWS world, değilse pkg_sim/worlds/<name>.world
    our_world_file = PathJoinSubstitution([pkg_sim, 'worlds', [world_name, TextSubstitution(text='.world')]])

    # Gazebo — small_warehouse seçilince AWS world (path varsa), yoksa kendi world'ümüz
    gazebo_our = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': our_world_file,
            'verbose': 'false',
        }.items(),
        condition=UnlessCondition(EqualsSubstitution(world_name, TextSubstitution(text='small_warehouse')))
    )
    # small_warehouse: AWS world varsa onu kullan, yoksa warehouse.world'a düş
    world_for_small = aws_world_path if aws_world_path else os.path.join(pkg_sim, 'worlds', 'warehouse.world')
    gazebo_aws = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': TextSubstitution(text=world_for_small),
            'verbose': 'false',
        }.items(),
        condition=IfCondition(EqualsSubstitution(world_name, TextSubstitution(text='small_warehouse')))
    )

    robot_description = Command(['xacro ', urdf_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }]
    )

    # Robotu Gazebo'ya spawn et (dünya yüklenene kadar 8 saniye bekle)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mecanum_amr',
            '-x', '1.767322',
            '-y', '-9.328291',
            '-z', '0.124316',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '1.581301',
        ],
        output='screen'
    )
    delayed_spawn = TimerAction(period=8.0, actions=[spawn_entity])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World: warehouse, minimal, empty, small_warehouse (AWS)'),
        set_model_path,
        gazebo_our,
        gazebo_aws,
        rsp,
        delayed_spawn,
    ])

