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
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcel',
    maintainer_email='mstemmeler@gmx.de',
    description='ROS 2 application for Spot inspection and probing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fault_detector_ui = fault_detector_spot.ui.fault_detector_ui:main',
            'bt_runner = fault_detector_spot.application.behaviour_tree.runner:main',
            'record_manager = fault_detector_spot.application.recording.record_manager_node:main',
            'nav2_cmd_vel_gate = fault_detector_spot.navigation.ros.nav2_cmd_vel_gate:main',
            'application_api = '
            'fault_detector_spot.application.api.application_api_node:main',
            'available_frames_publisher = '
            'fault_detector_spot.ui.ros.available_frames_publisher:main',
            'lidar_self_filter = fault_detector_spot.mapping.ros.lidar_self_filter:main',
        ],
    },
)
