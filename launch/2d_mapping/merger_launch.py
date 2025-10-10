from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # List of input camera topics
    camera_topics = [
        '/depth_registered/back/points',
        '/depth_registered/frontleft/points',
        '/depth_registered/frontright/points',
        '/depth_registered/left/points',
        '/depth_registered/right/points',
    ]

    nodes = []

    # Launch one transformer per camera
    for topic in camera_topics:
        nodes.append(
            Node(
                package='fault_detector_spot',
                executable='pointcloud_transformer',
                name=f'{topic.split("/")[1]}_transformer',
                parameters=[
                    {'input_topic': topic},
                    {'base_frame': 'base_link'},
                    {'output_suffix': '_base'}
                ],
                output='screen'
            )
        )

    # Launch merger node
    merged_topics = [t + '_base' for t in camera_topics]
    nodes.append(
        Node(
            package='fault_detector_spot',
            executable='pointcloud_merger',
            name='pointcloud_merger',
            parameters=[
                {'input_topics': merged_topics},
                {'output_topic': '/merged_cloud'},
                {'base_frame': 'base_link'}
            ],
            output='screen'
        )
    )

    return LaunchDescription(nodes)