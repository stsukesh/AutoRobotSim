"""
Test URDF and Xacro Consistency & XML Structure
"""

import os
import xml.etree.ElementTree as ET
import pytest

URDF_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../ros2_ws/src/auto_robot_description/urdf'))


def test_xacro_files_exist():
    expected_files = [
        'robot.urdf.xacro',
        'robot_core.xacro',
        'inertial_macros.xacro',
        'lidar.xacro',
        'camera.xacro',
        'imu.xacro',
        'gazebo_control.xacro'
    ]
    for filename in expected_files:
        path = os.path.join(URDF_DIR, filename)
        assert os.path.isfile(path), f"Missing expected Xacro file: {filename}"


def test_xml_syntax_validity():
    """Verify all xacro files are well-formed XML."""
    for filename in os.listdir(URDF_DIR):
        if filename.endswith('.xacro'):
            path = os.path.join(URDF_DIR, filename)
            tree = ET.parse(path)
            root = tree.getroot()
            assert root is not None
            assert 'robot' in root.tag


def test_joint_and_link_hierarchy():
    """Ensure core links and joints exist in robot_core.xacro."""
    core_path = os.path.join(URDF_DIR, 'robot_core.xacro')
    tree = ET.parse(core_path)
    root = tree.getroot()

    link_names = [elem.get('name') for elem in root.findall('link')]
    joint_names = [elem.get('name') for elem in root.findall('joint')]

    assert 'base_footprint' in link_names
    assert 'base_link' in link_names
    assert 'chassis' in link_names
    assert 'left_wheel' in link_names
    assert 'right_wheel' in link_names

    assert 'base_footprint_joint' in joint_names
    assert 'left_wheel_joint' in joint_names
    assert 'right_wheel_joint' in joint_names


if __name__ == '__main__':
    pytest.main(['-v', __file__])
