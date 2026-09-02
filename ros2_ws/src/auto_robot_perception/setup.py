import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_robot_perception'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Developer',
    maintainer_email='robotics@example.com',
    description='OpenCV perception and visual servoing for autonomous robotics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_detector_node = auto_robot_perception.visual_detector_node:main',
            'visual_servoing_node = auto_robot_perception.visual_servoing_node:main',
        ],
    },
)
