from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import SetRemap


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    launch_dir = os.path.dirname(__file__)
    map_dir = os.path.abspath(
        os.path.join(launch_dir, "..", "..", "..", "maps", "Hallway_map.yaml")
    )
    map_dir = map_dir.split("/install")[0]
    map_dir = os.path.join(map_dir, "maps", "Hallway_map.yaml")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=map_dir,
        description="Full path to map yaml file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("anscer_navigation"), "config", "nav2_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # Bringup Nav2
    bringup_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel',dst='/diff_drive_controller/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nav2_bringup"),
                                "launch",
                                "bringup_launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml_file,
                    "params_file": params_file,
                }.items(),
            )
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            bringup_launch,
        ]
    )
