import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Developer',
    maintainer_email='robotics@example.com',
    description='SLAM and Nav2 autonomous navigation stack configuration and mission orchestrator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_commander = auto_robot_navigation.mission_commander:main',
        ],
    },
)
