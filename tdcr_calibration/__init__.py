"""
TDCR Tendon Calibration Library

Automated tendon calibration system for tendon-driven continuum robots
using Intel RealSense camera and marker tracking.
"""

from .auto_tensioner import AutoTensioner
from .realsense_tracker import RealSenseTracker
from .config import CalibrationConfig, RealSenseConfig, MotionRecordingConfig

__version__ = "0.1.0"

__all__ = [
    "AutoTensioner",
    "RealSenseTracker",
    "CalibrationConfig",
    "RealSenseConfig",
    "MotionRecordingConfig",
]
