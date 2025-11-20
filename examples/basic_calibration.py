#!/usr/bin/env python3
"""
Basic calibration example for a simple 3-tendon TDCR.

This example demonstrates the minimal setup needed to calibrate
a tendon-driven continuum robot with default settings.
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tdcr_calibration import AutoTensioner, RealSenseTracker, CalibrationConfig, RealSenseConfig


def main():
    """Run basic calibration for a 3-tendon robot."""

    # Robot configuration
    calibration_config = CalibrationConfig(
        servo_ids=[1, 2, 3],              # Your servo IDs
        spool_radii_mm=10.0,              # Spool radius (uniform for all)
        servo_directions=[1, 1, 1],       # All servos same direction
        device_name="/dev/ttyUSB0",       # Serial port for Dynamixel

        # Calibration parameters (using defaults)
        loosen_amount_mm=5.0,             # How much to loosen before calibrating
        movement_threshold_mm=0.5,        # Displacement threshold for tension detection
        movement_countback_mm=0.0,        # Compensation amount

        # Enable direction verification
        verify_loosen_directions=True,
    )

    # RealSense tracker configuration (using defaults for ftdcr_v4)
    # Adjust these if your camera is mounted differently
    rs_config = RealSenseConfig()

    print("="*60)
    print("Basic TDCR Calibration")
    print("="*60)
    print(f"Servo IDs: {calibration_config.servo_ids}")
    print(f"Device: {calibration_config.device_name}")
    print()

    try:
        # Initialize tracker
        print("Initializing RealSense tracker...")
        tracker = RealSenseTracker(
            camera_width=rs_config.camera_width,
            camera_height=rs_config.camera_height,
            fps=rs_config.fps,
            hsv_range=rs_config.hsv_range,
            T_base_cam=rs_config.T_base_cam,
            smooth_alpha_xy=rs_config.smooth_alpha_xy,
            smooth_alpha_z=rs_config.smooth_alpha_z,
            enable_filters=rs_config.enable_filters
        )
        tracker.start()

        # Initialize auto-tensioner
        print("\nInitializing calibration system...")
        tensioner = AutoTensioner(tracker, calibration_config)

        # Run calibration
        print("\nStarting calibration procedure...")
        zero_positions = tensioner.calibrate()

        print("\n" + "="*60)
        print("Calibration Results")
        print("="*60)
        print(f"Zero positions: {zero_positions}")
        print("\nCalibration complete! Your robot is now calibrated.")

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
    except Exception as e:
        print(f"\n\nError during calibration: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'tensioner' in locals():
            tensioner.cleanup()
        elif 'tracker' in locals():
            tracker.stop()


if __name__ == "__main__":
    main()
