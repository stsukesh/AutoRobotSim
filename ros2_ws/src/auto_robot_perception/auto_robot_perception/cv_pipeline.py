"""
OpenCV Visual Perception Pipeline
Implements robust color-space segmentation, contour extraction,
pinhole camera geometric estimation (distance, bearing, elevation),
and visual telemetry overlay generation.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
import numpy as np
import cv2


@dataclass
class DetectionResult:
    detected: bool
    target_color: str
    centroid_px: Tuple[int, int]
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    contour_area: float
    distance_m: float
    bearing_rad: float
    elevation_rad: float
    position_camera_frame: Tuple[float, float, float]  # (X, Y, Z) in meters


class VisualPerceptionPipeline:
    def __init__(
        self,
        target_color: str = 'red',
        min_contour_area: float = 400.0,
        target_real_height_m: float = 0.6,
        target_real_width_m: float = 0.3,
        camera_fx: float = 554.25,
        camera_fy: float = 554.25,
        camera_cx: float = 320.0,
        camera_cy: float = 240.0
    ):
        self.target_color = target_color.lower()
        self.min_contour_area = min_contour_area
        self.target_real_height_m = target_real_height_m
        self.target_real_width_m = target_real_width_m

        # Intrinsic camera parameters
        self.fx = camera_fx
        self.fy = camera_fy
        self.cx = camera_cx
        self.cy = camera_cy

        # HSV Color ranges
        self.color_ranges: Dict[str, list] = {
            'red': [
                (np.array([0, 100, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
                (np.array([160, 100, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
            ],
            'blue': [
                (np.array([100, 120, 50], dtype=np.uint8), np.array([140, 255, 255], dtype=np.uint8))
            ],
            'green': [
                (np.array([35, 80, 50], dtype=np.uint8), np.array([85, 255, 255], dtype=np.uint8))
            ],
            'yellow': [
                (np.array([20, 100, 100], dtype=np.uint8), np.array([30, 255, 255], dtype=np.uint8))
            ]
        }

        # Morphological kernels
        self.morph_kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.morph_kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    def update_intrinsics(self, fx: float, fy: float, cx: float, cy: float):
        """Update camera intrinsic matrix from CameraInfo topic."""
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def set_target_color(self, color_name: str):
        """Set active target color for segmentation."""
        if color_name.lower() in self.color_ranges:
            self.target_color = color_name.lower()

    def create_color_mask(self, hsv_image: np.ndarray) -> np.ndarray:
        """Create binary mask for configured color, handling HSV hue wraparound."""
        ranges = self.color_ranges.get(self.target_color, self.color_ranges['red'])
        mask = None
        for lower, upper in ranges:
            curr_mask = cv2.inRange(hsv_image, lower, upper)
            if mask is None:
                mask = curr_mask
            else:
                mask = cv2.bitwise_or(mask, curr_mask)

        # Apply morphological opening to remove small noise dots
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel_open)
        # Apply morphological closing to fill interior holes
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, self.morph_kernel_close)
        return mask_clean

    def process_frame(self, bgr_image: np.ndarray) -> Tuple[DetectionResult, np.ndarray]:
        """
        Process single BGR frame, detecting target, calculating metrics,
        and generating annotated visualization.
        """
        if bgr_image is None or bgr_image.size == 0:
            empty_res = DetectionResult(
                detected=False, target_color=self.target_color,
                centroid_px=(0, 0), bounding_box=(0, 0, 0, 0),
                contour_area=0.0, distance_m=0.0, bearing_rad=0.0,
                elevation_rad=0.0, position_camera_frame=(0.0, 0.0, 0.0)
            )
            return empty_res, bgr_image

        annotated_img = bgr_image.copy()
        height, width = bgr_image.shape[:2]

        # Convert to HSV color space
        hsv_img = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # Segment color
        mask = self.create_color_mask(hsv_img)

        # Extract contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        max_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_contour_area and area > max_area:
                max_area = area
                best_contour = cnt

        if best_contour is None:
            # Target not found
            res = DetectionResult(
                detected=False,
                target_color=self.target_color,
                centroid_px=(0, 0),
                bounding_box=(0, 0, 0, 0),
                contour_area=0.0,
                distance_m=0.0,
                bearing_rad=0.0,
                elevation_rad=0.0,
                position_camera_frame=(0.0, 0.0, 0.0)
            )
            # Render search overlay
            cv2.putText(
                annotated_img,
                f"SEARCHING FOR: {self.target_color.upper()} TARGET",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 165, 255),
                2,
                cv2.LINE_AA
            )
            return res, annotated_img

        # Centroid calculation via image moments
        moments = cv2.moments(best_contour)
        if moments['m00'] > 0:
            cx_px = int(moments['m10'] / moments['m00'])
            cy_px = int(moments['m01'] / moments['m00'])
        else:
            x, y, w, h = cv2.boundingRect(best_contour)
            cx_px = x + w // 2
            cy_px = y + h // 2

        # Bounding box
        x, y, w, h = cv2.boundingRect(best_contour)

        # Geometric Distance & Bearing Estimation
        # Distance calculation using pinhole projection model based on target height
        # Z = (fy * H_real) / h_pixels
        estimated_dist_h = (self.fy * self.target_real_height_m) / max(h, 1)
        estimated_dist_w = (self.fx * self.target_real_width_m) / max(w, 1)
        estimated_dist = 0.7 * estimated_dist_h + 0.3 * estimated_dist_w

        # Coordinate in camera frame (Z forward, X right, Y down)
        pos_z = float(estimated_dist)
        pos_x = float((cx_px - self.cx) * pos_z / self.fx)
        pos_y = float((cy_px - self.cy) * pos_z / self.fy)

        # Angles
        bearing_rad = float(np.arctan2(cx_px - self.cx, self.fx))
        elevation_rad = float(np.arctan2(cy_px - self.cy, self.fy))

        res = DetectionResult(
            detected=True,
            target_color=self.target_color,
            centroid_px=(cx_px, cy_px),
            bounding_box=(x, y, w, h),
            contour_area=float(max_area),
            distance_m=pos_z,
            bearing_rad=bearing_rad,
            elevation_rad=elevation_rad,
            position_camera_frame=(pos_x, pos_y, pos_z)
        )

        # --------------------------------------------------
        # Render Professional HUD / Telemetry on Image
        # --------------------------------------------------
        # Draw bounding box
        cv2.rectangle(annotated_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Draw centroid crosshair
        cross_size = 12
        cv2.line(annotated_img, (cx_px - cross_size, cy_px), (cx_px + cross_size, cy_px), (0, 0, 255), 2)
        cv2.line(annotated_img, (cx_px, cy_px - cross_size), (cx_px, cy_px + cross_size), (0, 0, 255), 2)
        cv2.circle(annotated_img, (cx_px, cy_px), 4, (0, 255, 255), -1)

        # Center reticle
        center_x, center_y = int(self.cx), int(self.cy)
        cv2.drawMarker(annotated_img, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

        # Vector line from center to target
        cv2.line(annotated_img, (center_x, center_y), (cx_px, cy_px), (255, 255, 0), 1, cv2.LINE_AA)

        # Overlay text
        bearing_deg = np.degrees(bearing_rad)
        cv2.putText(
            annotated_img,
            f"TARGET LOCKED: {self.target_color.upper()}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )
        cv2.putText(
            annotated_img,
            f"Dist: {pos_z:.2f} m | Bearing: {bearing_deg:+.1f} deg",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )
        cv2.putText(
            annotated_img,
            f"Cam Pos (X,Y,Z): ({pos_x:+.2f}, {pos_y:+.2f}, {pos_z:.2f}) m",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
            cv2.LINE_AA
        )

        return res, annotated_img
