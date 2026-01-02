from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_drone_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Autonomous warehouse drone navigation using ArUco markers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = warehouse_drone_nav.aruco_detector_node:main',
            'position_estimator = warehouse_drone_nav.position_estimator_node:main',
            'flight_controller = warehouse_drone_nav.flight_controller_node:main',
            'mission_manager = warehouse_drone_nav.mission_manager_node:main',
            'auto_startup = warehouse_drone_nav.auto_startup_node:main',
        ],
    },
)
