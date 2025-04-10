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
            'object_scanner = fault_detector_spot.object_scanner:main',
            'sim_tests = fault_detector_spot.sim_tests:main',
            'print_apriltags = fault_detector_spot.print_apriltags:main',
            'sync_image_camera = fault_detector_spot.sync_image_camera:main',
            'wasd_arm = fault_detector_spot.wasd_arm:main',
            'arm_goal_navigator = fault_detector_spot.arm_goal_navigator:main'
        ],
    },
)
