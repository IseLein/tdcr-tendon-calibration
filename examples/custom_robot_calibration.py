#!/usr/bin/env python3
"""
Template calibration script for custom TDCR configurations.

Copy this file and modify the parameters for your specific robot.
"""

import sys
import os
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tdcr_calibration import AutoTensioner, RealSenseTracker, CalibrationConfig, RealSenseConfig


def main():
    """Run calibration for a custom robot configuration."""

    # =================================================================
    # ROBOT CONFIGURATION - Modify these for your robot
    # =================================================================

    # Dynamixel servo configuration
    servo_ids = [1, 2, 3, 4, 5, 6]  # List your servo IDs here
    servo_directions = [1, 1, 1, 1, 1, 1]  # +1 or -1 for each servo
    spool_radii_mm = 10.0  # Can be a single value or list per servo
    device_name = "/dev/ttyUSB0"  # Your serial port

    # Calibration parameters
    # These can be single values (applied to all) or lists (per servo)
    loosen_amount_mm = 5.0  # How much to loosen before calibration
    movement_threshold_mm = 0.5  # Tip displacement indicating tension
    movement_countback_mm = 0.0  # Compensation for elasticity

    # =================================================================
    # REALSENSE CAMERA CONFIGURATION
    # =================================================================

    # Camera settings
    camera_width = 640
    camera_height = 480
    fps = 90

    # HSV color range for marker detection
    # For red markers: ((165, 70, 70), (15, 255, 255))
    # For green markers: ((40, 40, 40), (80, 255, 255))
    # For blue markers: ((100, 40, 40), (130, 255, 255))
    hsv_range = ((165, 70, 70), (15, 255, 255))

    # Camera to base transformation matrix
    # This 4x4 matrix transforms points from camera frame to robot base frame
    # Calibrate this for your specific camera mounting
    T_base_cam = np.array([
        [1.0, 0.0, 0.0, 0.0],  # Modify these values for your setup
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Smoothing parameters (0 = no smoothing, 1 = no update)
    smooth_alpha_xy = 0.5  # Lateral position smoothing
    smooth_alpha_z = 0.1   # Depth smoothing (usually more aggressive)

    # =================================================================
    # Create configuration objects
    # =================================================================

    calibration_config = CalibrationConfig(
        servo_ids=servo_ids,
        spool_radii_mm=spool_radii_mm,
        servo_directions=servo_directions,
        device_name=device_name,

        loosen_amount_mm=loosen_amount_mm,
        movement_threshold_mm=movement_threshold_mm,
        movement_countback_mm=movement_countback_mm,

        verify_loosen_directions=True,
        enable_interactive_adjustment=True,
    )

    rs_config = RealSenseConfig(
        camera_width=camera_width,
        camera_height=camera_height,
        fps=fps,
        hsv_range=hsv_range,
        T_base_cam=T_base_cam,
        smooth_alpha_xy=smooth_alpha_xy,
        smooth_alpha_z=smooth_alpha_z,
        enable_filters=True
    )

    # =================================================================
    # Run calibration
    # =================================================================

    print("="*60)
    print("Custom TDCR Calibration")
    print("="*60)
    print(f"Servo IDs: {servo_ids}")
    print(f"Device: {device_name}")
    print(f"Number of tendons: {len(servo_ids)}")
    print()

    proceed = input("Proceed with calibration? (y/n): ").lower()
    if proceed != 'y':
        print("Calibration aborted.")
        return

    try:
        # Initialize tracker
        print("\nInitializing RealSense tracker...")
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
