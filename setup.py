import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fault_detector_spot'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/fault_detector_spot/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcel',
    maintainer_email='mstemmeler@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fault_detector_ui = fault_detector_spot.behaviour_tree.ui_classes.fault_detector_ui:main',
            'bt_runner = fault_detector_spot.behaviour_tree.bt_runner:main',
            'record_manager = fault_detector_spot.behaviour_tree.record_manager_node:main',
            'pointcloud_merger = fault_detector_spot.behaviour_tree.nodes.mapping.pointcloud_merger:main',
            'pointcloud_republisher = fault_detector_spot.behaviour_tree.nodes.mapping.pointcloud_republisher:main',
            'rgb_resizer = fault_detector_spot.behaviour_tree.nodes.mapping.rgb_resizer:main',
            'nav2_cmd_vel_gate = fault_detector_spot.behaviour_tree.nodes.navigation.nav2_cmd_vel_gate:main',
            'rest_api_bridge = fault_detector_spot.behaviour_tree.rest_bridge:main',
            'available_frames_publisher = fault_detector_spot.behaviour_tree.ui_classes.available_frames_publisher:main',
        ],
    },
)