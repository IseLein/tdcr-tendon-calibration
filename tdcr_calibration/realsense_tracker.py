"""
RealSense-based marker tracking for tendon-driven continuum robots.

This module provides real-time 3D position tracking of a colored marker
using an Intel RealSense depth camera.
"""

import numpy as np
import pyrealsense2 as rs
import cv2
from typing import Optional, Tuple


class RealSenseTracker:
    """
    Tracks a colored marker in 3D space using Intel RealSense camera.

    The tracker uses HSV color filtering to detect a marker and provides
    its 3D position in either the camera frame or a transformed base frame.
    """

    def __init__(
        self,
        camera_width: int = 640,
        camera_height: int = 480,
        fps: int = 90,
        hsv_range: Tuple[Tuple[int, int, int], Tuple[int, int, int]] = ((165, 70, 70), (15, 255, 255)),
        T_base_cam: Optional[np.ndarray] = None,
        smooth_alpha_xy: float = 0.5,
        smooth_alpha_z: float = 0.1,
        enable_filters: bool = True
    ):
        """
        Initialize RealSense tracker.

        Args:
            camera_width: Camera resolution width in pixels
            camera_height: Camera resolution height in pixels
            fps: Camera frame rate
            hsv_range: HSV color range for marker detection ((H_min, S_min, V_min), (H_max, S_max, V_max))
                      Note: For red markers that wrap around 0/180, use values like (165, ...) to (15, ...)
            T_base_cam: 4x4 homogeneous transformation matrix from camera to base frame
            smooth_alpha_xy: Exponential smoothing factor for XY position (0=no smoothing, 1=no update)
            smooth_alpha_z: Exponential smoothing factor for Z/depth (0=no smoothing, 1=no update)
            enable_filters: Enable RealSense post-processing filters for better depth quality
        """
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.fps = fps
        self.hsv_min, self.hsv_max = hsv_range
        self.smooth_alpha_xy = smooth_alpha_xy
        self.smooth_alpha_z = smooth_alpha_z
        self.enable_filters = enable_filters

        # Transformation matrix (identity if not provided)
        if T_base_cam is None:
            self.T_base_cam = np.eye(4)
        else:
            self.T_base_cam = np.array(T_base_cam)

        # RealSense pipeline
        self.pipeline = None
        self.align = None
        self.intrinsics = None

        # Smoothing state
        self.last_position = None

        # Filters
        self.spatial_filter = None
        self.temporal_filter = None
        self.hole_filling_filter = None

    def start(self):
        """Start the RealSense camera and initialize filters."""
        print(f"Starting RealSense camera at {self.camera_width}x{self.camera_height} @ {self.fps}fps")

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, self.fps)

        # Start pipeline
        profile = self.pipeline.start(config)

        # Get camera intrinsics
        color_stream = profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        # Create alignment object (align depth to color)
        self.align = rs.align(rs.stream.color)

        # Initialize filters if enabled
        if self.enable_filters:
            self.spatial_filter = rs.spatial_filter()
            self.spatial_filter.set_option(rs.option.filter_magnitude, 2)
            self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
            self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20)

            self.temporal_filter = rs.temporal_filter()
            self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
            self.temporal_filter.set_option(rs.option.filter_smooth_delta, 20)

            self.hole_filling_filter = rs.hole_filling_filter()

        print("RealSense camera started successfully")

    def stop(self):
        """Stop the RealSense camera."""
        if self.pipeline:
            self.pipeline.stop()
            print("RealSense camera stopped")

    def _apply_filters(self, depth_frame):
        """Apply post-processing filters to depth frame."""
        if not self.enable_filters:
            return depth_frame

        filtered = depth_frame
        if self.spatial_filter:
            filtered = self.spatial_filter.process(filtered)
        if self.temporal_filter:
            filtered = self.temporal_filter.process(filtered)
        if self.hole_filling_filter:
            filtered = self.hole_filling_filter.process(filtered)

        return filtered

    def _detect_marker(self, color_image: np.ndarray) -> Optional[Tuple[int, int]]:
        """
        Detect marker in color image using HSV color filtering.

        Args:
            color_image: BGR color image from camera

        Returns:
            (x, y) pixel coordinates of marker centroid, or None if not detected
        """
        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Handle red color wrapping around 0/180 in HSV
        if self.hsv_min[0] > self.hsv_max[0]:  # Red wraps around
            # Create two masks and combine them
            mask1 = cv2.inRange(hsv, np.array([self.hsv_min[0], self.hsv_min[1], self.hsv_min[2]]),
                               np.array([180, self.hsv_max[1], self.hsv_max[2]]))
            mask2 = cv2.inRange(hsv, np.array([0, self.hsv_min[1], self.hsv_min[2]]),
                               np.array([self.hsv_max[0], self.hsv_max[1], self.hsv_max[2]]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # Normal color range
            mask = cv2.inRange(hsv, np.array(self.hsv_min), np.array(self.hsv_max))

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Find largest contour (assumed to be the marker)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return (cx, cy)

    def _smooth_position(self, new_position: np.ndarray) -> np.ndarray:
        """
        Apply exponential smoothing to position.

        Uses different smoothing factors for XY (lateral) and Z (depth).
        """
        if self.last_position is None:
            self.last_position = new_position
            return new_position

        smoothed = np.zeros(3)
        # Smooth XY with one alpha
        smoothed[0] = self.smooth_alpha_xy * self.last_position[0] + (1 - self.smooth_alpha_xy) * new_position[0]
        smoothed[1] = self.smooth_alpha_xy * self.last_position[1] + (1 - self.smooth_alpha_xy) * new_position[1]
        # Smooth Z with another alpha (usually more aggressive for stable depth)
        smoothed[2] = self.smooth_alpha_z * self.last_position[2] + (1 - self.smooth_alpha_z) * new_position[2]

        self.last_position = smoothed
        return smoothed

    def get_position(self, in_base_frame: bool = True) -> Optional[np.ndarray]:
        """
        Get 3D position of the marker.

        Args:
            in_base_frame: If True, return position in base frame using T_base_cam transform.
                          If False, return position in camera frame.

        Returns:
            3D position [x, y, z] in meters, or None if marker not detected
        """
        if not self.pipeline:
            raise RuntimeError("Camera not started. Call start() first.")

        # Wait for frames
        frames = self.pipeline.wait_for_frames()

        # Align depth to color
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None

        # Apply filters to depth
        depth_frame = self._apply_filters(depth_frame)

        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Detect marker in color image
        marker_pixel = self._detect_marker(color_image)
        if marker_pixel is None:
            return None

        px, py = marker_pixel

        # Get depth at marker location
        depth = depth_frame.get_distance(px, py)
        if depth == 0:
            return None  # Invalid depth

        # Deproject pixel to 3D point in camera frame
        point_3d_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px, py], depth)

        # Convert to numpy array
        position_cam = np.array(point_3d_cam)

        # Apply smoothing
        position_cam = self._smooth_position(position_cam)

        if not in_base_frame:
            return position_cam

        # Transform to base frame
        position_cam_homogeneous = np.array([position_cam[0], position_cam[1], position_cam[2], 1.0])
        position_base_homogeneous = self.T_base_cam @ position_cam_homogeneous
        position_base = position_base_homogeneous[:3]

        return position_base

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
