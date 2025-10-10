# fault_detector_spot/launch/nav2_spot_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # prefer installed package config, fall back to workspace src config (dev)
    pkg_share = get_package_share_directory('fault_detector_spot')
    install_config = os.path.join(pkg_share, 'config', 'nav2_spot_params.yaml')

    workspace_config = os.path.join(
        os.getcwd(), 'src', 'fault_detector_spot', 'config', 'nav2_spot_params.yaml'
    )

    if os.path.exists(install_config):
        default_params = install_config
    elif os.path.exists(workspace_config):
        default_params = workspace_config
    else:
        # last resort: default to install path (launch will warn)
        default_params = install_config

    params_file = LaunchConfiguration('params_file')

    lifecycle_delay_sec = 6.0  # increase if RTAB-Map takes longer to publish TF/topics

    ld = LaunchDescription()
    ld.add_action(LogInfo(msg=['Nav2 params default: ', default_params]))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the Nav2 params file (nav2_spot_params.yaml)'
    ))

    # Controller server
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    ))

    # Planner server
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    ))

    # BT navigator
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    ))

    # Behavior / recovery server (Spin/BackUp/Wait)
    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    ))

    # Lifecycle manager (delayed start to reduce race with RTAB-Map)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server'
            ]
        }],
    )

    ld.add_action(TimerAction(period=lifecycle_delay_sec, actions=[lifecycle_manager_node]))

    return ld