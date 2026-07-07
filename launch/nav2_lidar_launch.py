import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg = get_package_share_directory("fault_detector_spot")


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.expanduser("~/.ros/slam_map.yaml"),
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg, "config", "nav2_lidar_params.yaml"),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("params_file", LaunchConfiguration("params_file")),
            ("map", LaunchConfiguration("map")),
        ],
    )

    cmd_vel_gate_node = Node(
        package="fault_detector_spot",
        executable="nav2_cmd_vel_gate",
        name="nav2_cmd_vel_gate",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        nav2,
        cmd_vel_gate_node,
    ])