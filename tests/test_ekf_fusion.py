"""
Test EKF Configuration, Covariance Dimensions, and Math Consistency
"""

import os
import yaml
import numpy as np
import pytest

CONFIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../ros2_ws/src/auto_robot_localization/config'))


def test_ekf_yaml_structure():
    yaml_path = os.path.join(CONFIG_DIR, 'ekf.yaml')
    assert os.path.isfile(yaml_path)

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    assert 'ekf_filter_node' in config
    params = config['ekf_filter_node']['ros__parameters']

    assert params['two_d_mode'] is True
    assert params['publish_tf'] is True
    assert params['odom_frame'] == 'odom'
    assert params['base_link_frame'] == 'base_footprint'


def test_ekf_covariance_matrices():
    yaml_path = os.path.join(CONFIG_DIR, 'ekf.yaml')
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config['ekf_filter_node']['ros__parameters']

    # State vector is 15-dimensional -> covariance must be 15x15 = 225 elements
    pnc = params['process_noise_covariance']
    assert len(pnc) == 225, f"Process noise covariance length must be 225 (15x15), got {len(pnc)}"

    cov_matrix = np.array(pnc).reshape((15, 15))
    # Covariance diagonal must be positive
    for i in range(15):
        assert cov_matrix[i, i] >= 0.0, f"Diagonal element [{i},{i}] cannot be negative"


def test_odom_imu_config_booleans():
    yaml_path = os.path.join(CONFIG_DIR, 'ekf.yaml')
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config['ekf_filter_node']['ros__parameters']

    odom0_cfg = params['odom0_config']
    imu0_cfg = params['imu0_config']

    assert len(odom0_cfg) == 15, "odom0_config must have 15 boolean flags"
    assert len(imu0_cfg) == 15, "imu0_config must have 15 boolean flags"

    # In 2D mode, odom should track x (0), y (1), vx (6), vyaw (11)
    assert odom0_cfg[0] is True
    assert odom0_cfg[1] is True

    # IMU should track yaw (5) and yaw velocity (11)
    assert imu0_cfg[5] is True
    assert imu0_cfg[11] is True


if __name__ == '__main__':
    pytest.main(['-v', __file__])
