from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('anscer_description')
    world_pkg_share = get_package_share_directory("aws_robomaker_hospital_world")
    model_path = (
        os.path.join(world_pkg_share, 'models')
        + ":"
        + os.path.join(world_pkg_share, 'fuel_models')
        + ":"
        + os.path.join(pkg_share, "meshes")
    )
    world_file = os.path.join(world_pkg_share, 'worlds', 'hospital.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'anscer_description.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'anscer_config.rviz')

    # Process xacro -> urdf
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Delayed Spawn Entity (wait until gazebo loads) (could use ExecuteProcess and OnStart as well but this works for now)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-y', '10.0'],
            output='screen'
        )]
    )

    # Controllers (wait until robot is spawned)
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        )]
    )

    diff_drive_controller_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            output="screen"
        )]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gazebo,
        rsp,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz
    ])
