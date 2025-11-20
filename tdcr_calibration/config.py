"""
Configuration dataclasses for TDCR tendon calibration.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Union, Tuple
import numpy as np


@dataclass
class RealSenseConfig:
    """
    Configuration for RealSense camera tracking.

    Attributes:
        camera_width: Camera resolution width in pixels
        camera_height: Camera resolution height in pixels
        fps: Camera frame rate
        hsv_range: HSV color range for marker detection ((H_min, S_min, V_min), (H_max, S_max, V_max))
                  For red markers that wrap around 0/180, use values like ((165, 70, 70), (15, 255, 255))
        T_base_cam: 4x4 homogeneous transformation matrix from camera to base frame
        smooth_alpha_xy: Exponential smoothing factor for XY position (0=no smoothing, 1=no update)
        smooth_alpha_z: Exponential smoothing factor for Z/depth (0=no smoothing, 1=no update)
        enable_filters: Enable RealSense post-processing filters
    """
    camera_width: int = 640
    camera_height: int = 480
    fps: int = 90
    hsv_range: Tuple[Tuple[int, int, int], Tuple[int, int, int]] = ((165, 70, 70), (15, 255, 255))
    T_base_cam: np.ndarray = field(default_factory=lambda: np.array([
        [1.0, 0.0, 0.0, -0.009],
        [0.0, 0.9659258, -0.2588190, 0.08833],
        [0.0, 0.2588190, 0.9659258, -0.05023],
        [0.0, 0.0, 0.0, 1.0]
    ]))
    smooth_alpha_xy: float = 0.5
    smooth_alpha_z: float = 0.1
    enable_filters: bool = True


@dataclass
class CalibrationConfig:
    """
    Configuration for automated tendon calibration.

    Attributes:
        servo_ids: List of Dynamixel servo IDs controlling the tendons
        spool_radii_mm: Radius of spool in mm (single value or list per servo)
        servo_directions: Direction multipliers for each servo (+1 or -1, None for all +1)
        device_name: Serial port device name for Dynamixel communication

        loosen_amount_mm: How much to loosen each tendon before calibration (single value or list)
        movement_threshold_mm: Displacement threshold indicating tension engagement (single value or list)
        movement_countback_mm: Compensation for tendon elasticity (single value or list)
        tightening_increment_mm: Increment for gradual tightening during calibration
        tightening_delay_s: Delay between tightening steps
        settling_time_s: Time to wait for robot to settle after movements

        verify_loosen_directions: Whether to verify servo loosen directions before calibration

        verify_tracking_duration_s: Duration to verify marker tracking at startup
        verify_tracking_sample_rate_hz: Sample rate for tracking verification

        enable_interactive_adjustment: Allow post-calibration manual adjustment
        enable_direction_check: Offer interactive direction checking
    """
    # Robot configuration
    servo_ids: List[int] = field(default_factory=list)
    spool_radii_mm: Union[float, List[float]] = 10.0
    servo_directions: Optional[List[float]] = None
    device_name: str = "/dev/ttyUSB0"

    # Calibration parameters
    loosen_amount_mm: Union[float, List[float]] = 5.0
    movement_threshold_mm: Union[float, List[float]] = 0.5
    movement_countback_mm: Union[float, List[float]] = 0.0
    tightening_increment_mm: float = 0.0025
    tightening_delay_s: float = 0.01
    settling_time_s: float = 1.0

    # Direction verification
    verify_loosen_directions: bool = True

    # Tracking verification
    verify_tracking_duration_s: float = 5.0
    verify_tracking_sample_rate_hz: float = 10.0

    # Interactive features
    enable_interactive_adjustment: bool = True
    enable_direction_check: bool = False

    def __post_init__(self):
        """Validate and normalize configuration after initialization."""
        if not self.servo_ids:
            raise ValueError("servo_ids cannot be empty")

        num_servos = len(self.servo_ids)

        # Convert scalar values to lists
        if isinstance(self.spool_radii_mm, (int, float)):
            self.spool_radii_mm = [float(self.spool_radii_mm)] * num_servos

        if isinstance(self.loosen_amount_mm, (int, float)):
            self.loosen_amount_mm = [float(self.loosen_amount_mm)] * num_servos

        if isinstance(self.movement_threshold_mm, (int, float)):
            self.movement_threshold_mm = [float(self.movement_threshold_mm)] * num_servos

        if isinstance(self.movement_countback_mm, (int, float)):
            self.movement_countback_mm = [float(self.movement_countback_mm)] * num_servos

        # Validate list lengths
        if len(self.spool_radii_mm) != num_servos:
            raise ValueError(f"spool_radii_mm must have {num_servos} values")

        if len(self.loosen_amount_mm) != num_servos:
            raise ValueError(f"loosen_amount_mm must have {num_servos} values")

        if len(self.movement_threshold_mm) != num_servos:
            raise ValueError(f"movement_threshold_mm must have {num_servos} values")

        if len(self.movement_countback_mm) != num_servos:
            raise ValueError(f"movement_countback_mm must have {num_servos} values")

        # Set default servo directions if not provided
        if self.servo_directions is None:
            self.servo_directions = [1.0] * num_servos

        if len(self.servo_directions) != num_servos:
            raise ValueError(f"servo_directions must have {num_servos} values")


@dataclass
class MotionRecordingConfig:
    """
    Configuration for motion recording and repeatability testing.

    Attributes:
        amplitude_mm: Maximum amplitude for motions
        sweep_duration_s: Duration for each individual coordinate sweep
        pattern_duration_s: Duration for each circle/square/spiral pattern
        num_repetitions: Number of times to repeat the entire sequence
        control_rate_hz: Control loop rate for servo commands
        recording_rate_hz: Data recording rate

        tendon_distance_mm: Distance from tendon to backbone center (for kinematics)
        angle_offset_rad_ccw: Angular offset of tendons in each segment
    """
    amplitude_mm: float = 3.0
    sweep_duration_s: float = 10.0
    pattern_duration_s: float = 15.0
    num_repetitions: int = 3
    control_rate_hz: float = 50.0
    recording_rate_hz: float = 10.0

    # Kinematics parameters
    tendon_distance_mm: float = 4.5
    angle_offset_rad_ccw: np.ndarray = field(default_factory=lambda: np.array([0.0, np.pi/6, np.pi/3]))
