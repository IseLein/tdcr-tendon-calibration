#!/usr/bin/env python3
"""
Calibration script for ftdcr_v4 robot.

This is the complete calibration script for the 9-tendon (3 segments × 3 tendons)
ftdcr_v4 robot with custom per-servo parameters.
"""

import sys
import os
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tdcr_calibration import AutoTensioner, RealSenseTracker, CalibrationConfig, RealSenseConfig


def main():
    """Run calibration for ftdcr_v4 robot."""

    # ftdcr_v4 specific configuration
    device_name = '/dev/tty.usbserial-FT94F4DY'
    servo_ids = [3, 1, 10, 7, 5, 6, 11, 9, 2]
    servo_directions = [-1, -1, 1, -1, -1, 1, -1, -1, 1]
    spool_radii_mm = 10.0

    # Per-servo calibration parameters
    loosen_amounts = [10.0] * len(servo_ids)
    movement_thresholds = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
    movement_countbacks = [2.0] * len(servo_ids)

    calibration_config = CalibrationConfig(
        servo_ids=servo_ids,
        spool_radii_mm=spool_radii_mm,
        servo_directions=servo_directions,
        device_name=device_name,

        loosen_amount_mm=loosen_amounts,
        movement_threshold_mm=movement_thresholds,
        movement_countback_mm=movement_countbacks,

        verify_loosen_directions=True,
        enable_interactive_adjustment=True,
        enable_direction_check=False,  # Set to True if you want interactive direction checking
    )

    # RealSense configuration for ftdcr_v4
    rs_config = RealSenseConfig(
        camera_width=640,
        camera_height=480,
        fps=90,
        hsv_range=((165, 70, 70), (15, 255, 255)),  # Red/pink marker
        T_base_cam=np.array([
            [1.0, 0.0, 0.0, -0.009],
            [0.0, 0.9659258, -0.2588190, 0.08833],
            [0.0, 0.2588190, 0.9659258, -0.05023],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        smooth_alpha_xy=0.5,
        smooth_alpha_z=0.1,
        enable_filters=True
    )

    print("="*60)
    print("ftdcr_v4 Calibration")
    print("="*60)
    print(f"Servo IDs: {servo_ids}")
    print(f"Device: {device_name}")
    print("\nParameters for each servo:")
    print("-" * 50)
    print(f"{'Servo ID':<10} {'Loosen (mm)':<15} {'Threshold (mm)':<15} {'Countback (mm)':<15}")
    print("-" * 50)
    for i, sid in enumerate(servo_ids):
        print(f"{sid:<10} {loosen_amounts[i]:<15.2f} {movement_thresholds[i]:<15.2f} {movement_countbacks[i]:<15.2f}")
    print("-" * 50)
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

        # Optional: Ask about motion recording
        print("\n" + "="*60)
        print("Motion Recording (Optional)")
        print("="*60)
        print("You can now record motion data for repeatability analysis.")
        print("Note: This requires the kinematics module from crl_tdcr_teleop")
        print()

        record = input("Would you like to record motion data? (y/n): ").lower()
        if record == 'y':
            print("\nMotion recording not yet implemented in this version.")
            print("See the original auto_tension_rs_marker.py for motion recording features.")

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
