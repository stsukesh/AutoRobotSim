"""
Test Navigation Waypoints and Transformation Helpers
"""

import math
import pytest


def euler_to_quaternion(yaw_rad: float):
    """Calculate quaternion z and w from planar yaw."""
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw


def test_quaternion_normalization():
    """Verify quaternion magnitude equals 1.0 for arbitrary yaw angles."""
    test_angles = [0.0, math.pi / 4, math.pi / 2, math.pi, -math.pi / 2, 2 * math.pi]
    for yaw in test_angles:
        qx, qy, qz, qw = euler_to_quaternion(yaw)
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        assert abs(norm - 1.0) < 1e-6


def test_waypoint_arena_bounds():
    """Verify all predefined inspection waypoints lie inside the 10x10 arena bounds."""
    waypoints = [
        {'name': 'WP1', 'x': 1.0, 'y': 0.0},
        {'name': 'WP2', 'x': -2.2, 'y': 2.5},
        {'name': 'WP3', 'x': 2.2, 'y': -2.5},
        {'name': 'WP4', 'x': 0.0, 'y': -1.5}
    ]
    for wp in waypoints:
        assert -4.5 <= wp['x'] <= 4.5, f"Waypoint {wp['name']} X outside navigable boundary"
        assert -4.5 <= wp['y'] <= 4.5, f"Waypoint {wp['name']} Y outside navigable boundary"


if __name__ == '__main__':
    pytest.main(['-v', __file__])
