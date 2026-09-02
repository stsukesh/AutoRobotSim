"""
Unit Tests for OpenCV Visual Perception Pipeline
Tests color segmentation, geometric pinhole distance/bearing estimation,
noise rejection, and overlay rendering without requiring ROS/Gazebo.
"""

import sys
import os
import math
import numpy as np
import cv2
import pytest

# Add package directory to python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../ros2_ws/src/auto_robot_perception')))

from auto_robot_perception.cv_pipeline import VisualPerceptionPipeline, DetectionResult


@pytest.fixture
def perception_pipeline():
    return VisualPerceptionPipeline(
        target_color='red',
        min_contour_area=200.0,
        target_real_height_m=0.6,
        target_real_width_m=0.3,
        camera_fx=500.0,
        camera_fy=500.0,
        camera_cx=320.0,
        camera_cy=240.0
    )


def test_no_target_detection(perception_pipeline):
    """Test perception behavior on a blank black image."""
    blank_img = np.zeros((480, 640, 3), dtype=np.uint8)
    result, annotated = perception_pipeline.process_frame(blank_img)

    assert result.detected is False
    assert result.contour_area == 0.0
    assert result.distance_m == 0.0
    assert annotated.shape == (480, 640, 3)


def test_red_target_detection_and_geometry(perception_pipeline):
    """
    Test detection of a synthesized red target at known pixel size & position,
    and verify pinhole distance and bearing math.
    """
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Place a pure RED rectangle at center
    # Expected distance: Z = fy * H_real / h_pixels
    # If h_pixels = 100, Z = 500 * 0.6 / 100 = 3.0 meters
    h_px = 100
    w_px = 50
    center_x = 320
    center_y = 240

    top_left_x = center_x - w_px // 2
    top_left_y = center_y - h_px // 2

    # Draw pure red in BGR: (0, 0, 255)
    cv2.rectangle(img, (top_left_x, top_left_y), (top_left_x + w_px, top_left_y + h_px), (0, 0, 255), -1)

    result, annotated = perception_pipeline.process_frame(img)

    assert result.detected is True
    assert result.target_color == 'red'
    assert result.contour_area > 4000.0

    # Centroid should be within 2 pixels of center
    cx, cy = result.centroid_px
    assert abs(cx - center_x) <= 2
    assert abs(cy - center_y) <= 2

    # Bearing at image center should be approximately 0.0 rad
    assert abs(result.bearing_rad) < 0.05

    # Distance should be approximately 3.0m (within 10% tolerance)
    assert 2.7 <= result.distance_m <= 3.3


def test_off_center_target_bearing(perception_pipeline):
    """Test bearing angle calculation for off-center target."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Place target to the right (x=480, y=240) -> delta_x = 160 px
    # Expected bearing = arctan(160 / 500) = ~0.309 rad (~17.7 deg)
    h_px = 80
    w_px = 40
    target_x = 480
    target_y = 240

    cv2.rectangle(
        img,
        (target_x - w_px // 2, target_y - h_px // 2),
        (target_x + w_px // 2, target_y + h_px // 2),
        (0, 0, 255),
        -1
    )

    result, _ = perception_pipeline.process_frame(img)

    assert result.detected is True
    expected_bearing = math.atan2(160.0, 500.0)
    assert abs(result.bearing_rad - expected_bearing) < 0.05
    assert result.position_camera_frame[0] > 0.0  # Target is to the right (+X in camera optical frame)


def test_multi_color_switching(perception_pipeline):
    """Test changing target color to blue and detecting blue landmark."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Draw blue rectangle: BGR (255, 0, 0)
    cv2.rectangle(img, (200, 200), (280, 320), (255, 0, 0), -1)

    # First test: Searching for red should NOT detect blue
    perception_pipeline.set_target_color('red')
    res_red, _ = perception_pipeline.process_frame(img)
    assert res_red.detected is False

    # Second test: Switch to blue -> should detect blue!
    perception_pipeline.set_target_color('blue')
    res_blue, _ = perception_pipeline.process_frame(img)
    assert res_blue.detected is True
    assert res_blue.target_color == 'blue'


def test_noise_filtering(perception_pipeline):
    """Test that small noisy speckles under min_contour_area are ignored."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Draw small red dots (5x5 pixels = 25 px area < 200 min_contour_area)
    cv2.rectangle(img, (50, 50), (55, 55), (0, 0, 255), -1)
    cv2.rectangle(img, (150, 150), (154, 154), (0, 0, 255), -1)

    perception_pipeline.set_target_color('red')
    result, _ = perception_pipeline.process_frame(img)
    assert result.detected is False


if __name__ == '__main__':
    pytest.main(['-v', __file__])
