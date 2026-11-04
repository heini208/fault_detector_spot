import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

pkg = get_package_share_directory("fault_detector_spot")


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            pkg,
            "config",
            "nav2_lidar_params.yaml",
        ),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("params_file", LaunchConfiguration("params_file")),
            ("autostart", "true"),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        DeclareLaunchArgument(
            "velocity_topic", default_value="cmd_vel",
            description="Final velocity output; the application selects its private admission topic",
        ),
        GroupAction([
            # Node-qualified rules preserve the controller -> smoother input
            # chain while routing final output to the selected consumer.
            SetRemap(src="velocity_smoother:cmd_vel_smoothed",
                     dst=LaunchConfiguration("velocity_topic")),
            SetRemap(src="behavior_server:cmd_vel",
                     dst=LaunchConfiguration("velocity_topic")),
            nav2,
        ]),
    ])
