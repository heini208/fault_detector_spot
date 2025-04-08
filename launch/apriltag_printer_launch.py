from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to your custom YAML config file (my_tags.yaml)
    config_path = os.path.join(
        get_package_share_directory('fault_detector_spot'),  # Your package
        'config',  # Config folder
        'my_tags.yaml'  # Your custom YAML file
    )

    # Debug print to check the generated config path
    print(f"Using custom config file: {config_path}")

    # Return the launch description with the apriltag node
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            #remappings=[
            #   ('image_rect', '/SpotArm/gripper_camera/image_color'),
            #    ('camera_info', '/SpotArm/gripper_camera/camera_info')
            #],
            remappings=[
               ('image_rect', '/camera/hand/image'),
                ('camera_info', '/camera/hand/camera_info')
            ],
            parameters=[config_path, {'ros__log_level': 'debug'}],  # Use your custom YAML and set log level
            output='screen',  # This ensures logs are printed to the console
        )
    ])
