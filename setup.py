from setuptools import find_packages, setup
import os
from glob import glob

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
        (os.path.join('share', package_name, 'config'), ['config/my_tags.yaml']),
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
            'sim_tests = fault_detector_spot.sim_tests:main',
            'print_apriltags = fault_detector_spot.print_apriltags:main',
            'sync_image_camera = fault_detector_spot.sync_image_camera:main',
            'wasd_arm = fault_detector_spot.wasd_arm:main',
            'arm_goal_navigator = fault_detector_spot.arm_goal_navigator:main',
            'tag_follower = fault_detector_spot.tag_follower:main',
            'move_to_tag = fault_detector_spot.modules.move_to_tag_controller:main',
            'keyboard_check = fault_detector_spot.keyboard_check:main',
            'depth_at_tag = fault_detector_spot.depth_at_tag:main',
            'spot_keyboard = fault_detector_spot.spot_keyboard:main',
            'spot_tag_to_arm_goal = fault_detector_spot.modules.spot_tag_to_arm_goal:main',
            'spot_tag_to_arm_goal_ui = fault_detector_spot.modules.spot_tag_to_arm_goal_ui:main',
            'arm_controller = fault_detector_spot.modules.arm_controller:main',

        ],
    },
)
