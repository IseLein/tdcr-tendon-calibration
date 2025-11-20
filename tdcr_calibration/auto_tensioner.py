"""
Automated tendon calibration for tendon-driven continuum robots.

This module provides the core calibration logic for finding the zero tension
point of each tendon by progressively tightening and monitoring tip displacement.
"""

import sys
import os
import time
import numpy as np
from typing import Optional, List

# Add lib path for crl_tdcr_teleop imports
lib_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'lib')
if lib_path not in sys.path:
    sys.path.insert(0, lib_path)

try:
    from crl_tdcr_teleop.dynamixel_servo.multi_tendon_sync_rw import MultiTendonSyncRW
except ImportError:
    print("ERROR: Cannot import crl_tdcr_teleop. Make sure it's cloned in the lib/ directory")
    print(f"Expected location: {lib_path}/crl_tdcr_teleop")
    raise

from .config import CalibrationConfig, MotionRecordingConfig
from .realsense_tracker import RealSenseTracker
from .utils import draw_progress_bar


class AutoTensioner:
    """
    Automated tendon tensioning system for TDCRs.

    This class orchestrates the calibration process:
    1. Verify marker tracking
    2. Sanity check servo directions
    3. Find zero tension points for each tendon
    4. Apply final tensions with optional manual adjustment
    """

    def __init__(
        self,
        tracker: RealSenseTracker,
        config: CalibrationConfig
    ):
        """
        Initialize AutoTensioner.

        Args:
            tracker: Initialized RealSenseTracker instance
            config: Calibration configuration
        """
        self.tracker = tracker
        self.config = config

        # Ensure tracker is started
        if not self.tracker.pipeline:
            print("Starting tracker...")
            self.tracker.start()
            time.sleep(1)

        # Verify marker tracking before initializing servos
        if not self.verify_marker_tracking():
            self.cleanup()
            raise Exception("Marker verification failed!")

        # Initialize servo controller
        print("\nInitializing servo controller...")
        servo_ids = np.array(self.config.servo_ids, dtype=np.int32)
        spool_radii_mm = np.array(self.config.spool_radii_mm, dtype=np.float64)
        servo_dir = np.array(self.config.servo_directions, dtype=np.float64)

        self.servo_controller = MultiTendonSyncRW(
            servo_ids=servo_ids,
            zero_offsets_tick=None,
            spool_radii_mm=spool_radii_mm,
            device_name=self.config.device_name,
            servo_dir=servo_dir
        )
        self.servo_controller.set_tendons_speeds_mm_per_sec(
            np.ones(len(servo_ids), dtype=np.float64) * 1.0
        )
        self.servo_controller.set_zero_offsets_to_current_position()

        print("Initialization complete!")

    def verify_marker_tracking(
        self,
        duration: Optional[float] = None,
        sample_rate: Optional[float] = None
    ) -> bool:
        """
        Verify marker tracking for a few seconds and show relative movement.

        Args:
            duration: Verification duration in seconds (uses config if None)
            sample_rate: Sampling rate in Hz (uses config if None)

        Returns:
            True if tracking is stable and user confirms, False otherwise
        """
        if duration is None:
            duration = self.config.verify_tracking_duration_s
        if sample_rate is None:
            sample_rate = self.config.verify_tracking_sample_rate_hz

        print("\nVerifying marker tracking...")
        print("Please ensure the TDCR tip marker is visible to the camera.")

        # Get initial position
        initial_pos = None
        for _ in range(10):
            pos = self.get_marker_position()
            if pos is not None:
                initial_pos = pos
                break
            time.sleep(0.1)

        if initial_pos is None:
            print("ERROR: Cannot detect marker!")
            return False

        print("\nInitial position:", initial_pos)
        print("\nStreaming marker position (relative to initial position)...")
        print("Press Ctrl+C to stop early, or wait {:.0f} seconds".format(duration))

        try:
            start_time = time.time()
            while time.time() - start_time < duration:
                current_pos = self.get_marker_position()
                if current_pos is None:
                    print("\nERROR: Lost marker tracking!")
                    return False

                # Calculate and display relative position
                relative_pos = current_pos - initial_pos
                print(f"\rRelative position (mm): "
                      f"X: {relative_pos[0]:6.2f}, "
                      f"Y: {relative_pos[1]:6.2f}, "
                      f"Z: {relative_pos[2]:6.2f}", end="", flush=True)

                time.sleep(1.0 / sample_rate)

        except KeyboardInterrupt:
            print("\nMarker verification interrupted by user")

        print("\n\nMarker tracking verification complete.")
        proceed = input("Does the marker tracking look correct? (y/n): ").lower()
        return proceed == 'y'

    def get_marker_position(self) -> Optional[np.ndarray]:
        """
        Get position of the marker in millimeters.

        Returns:
            3D position [x, y, z] in mm, or None if not detected
        """
        position = self.tracker.get_position(in_base_frame=True)
        if position is not None:
            # Convert from meters to millimeters
            return np.array(position) * 1000.0
        return None

    def verify_loosen_directions(self) -> bool:
        """
        Verify that loosening commands actually loosen tendons.

        If movement > threshold during loosening, servo direction is wrong.
        Uses the minimum loosen_amount_mm and movement_threshold_mm from config.

        Returns:
            True if all directions are correct, False otherwise
        """
        # Use minimum values from config for conservative testing
        loosen_amount_mm = min(self.config.loosen_amount_mm)
        movement_threshold_mm = min(self.config.movement_threshold_mm)

        print("\n" + "="*60)
        print("DIRECTION VERIFICATION: Checking tendon loosen directions")
        print("="*60)
        print(f"Each tendon will be loosened by {loosen_amount_mm}mm")
        print(f"If tip moves > {movement_threshold_mm}mm, direction is WRONG")
        print()

        # Get initial position
        initial_pos = self.get_marker_position()
        if initial_pos is None:
            print("ERROR: Cannot detect marker!")
            return False

        # Store initial servo positions
        initial_servo_pos = self.servo_controller.get_tendons_mm()

        all_passed = True
        failed_servos = []

        for i, servo_id in enumerate(self.config.servo_ids):
            print(f"\nTesting servo {servo_id} (index {i})...")

            # Get position before loosening
            pos_before = self.get_marker_position()
            if pos_before is None:
                print("ERROR: Lost marker tracking!")
                all_passed = False
                break

            # Loosen this tendon only
            target_tensions = initial_servo_pos.copy()
            target_tensions[i] = initial_servo_pos[i] - loosen_amount_mm

            print(f"  Loosening by {loosen_amount_mm}mm...")
            self.servo_controller.async_set_tendons_mm(target_tensions)
            time.sleep(1.0)

            # Check position after loosening
            pos_after = self.get_marker_position()
            if pos_after is None:
                print("ERROR: Lost marker tracking!")
                all_passed = False
                break

            # Calculate movement
            movement = np.linalg.norm(pos_after - pos_before)
            print(f"  Tip movement: {movement:.2f}mm", end="")

            if movement > movement_threshold_mm:
                print(f" - FAIL! Movement exceeds threshold!")
                print(f"  This likely means servo {servo_id} is TIGHTENING instead of loosening!")
                all_passed = False
                failed_servos.append(servo_id)
            else:
                print(f" - OK")

            # Return to initial position
            self.servo_controller.async_set_tendons_mm(initial_servo_pos)
            time.sleep(0.5)

        # Return to initial position
        print("\nReturning all tendons to initial positions...")
        self.servo_controller.async_set_tendons_mm(initial_servo_pos)
        time.sleep(1.0)

        print("\n" + "="*60)
        if all_passed:
            print("DIRECTION VERIFICATION PASSED: All servo directions are correct!")
        else:
            print("DIRECTION VERIFICATION FAILED!")
            print(f"Failed servos: {failed_servos}")
            print("\nPlease check your servo_dir configuration!")
            print("The servo_dir values for these servos may need to be flipped.")
        print("="*60 + "\n")

        return all_passed

    def check_tendon_directions(self, debug: bool = False) -> bool:
        """
        Optional: Interactive check of tendon directions with user confirmation.

        Args:
            debug: Enable debug output

        Returns:
            True if check completes successfully
        """
        print("\nChecking tendon directions...")
        input("Ensure the TDCR is in its neutral position. Press Enter to continue...")

        for i, servo_id in enumerate(self.config.servo_ids):
            print(f"\nTesting servo {servo_id}:")
            input("Press Enter to start testing this tendon...")

            # Get initial position
            initial_pos = self.get_marker_position()
            if initial_pos is None:
                raise Exception("Cannot detect marker!")

            print("Loosening tendon... (monitoring for movement)")
            print("Press Ctrl+C to stop if movement is too large")

            target_tensions = np.zeros(len(self.config.servo_ids), dtype=np.float64)
            current_tension = 0
            movement_detected = False

            try:
                # Gradually loosen while monitoring position
                while current_tension > -5 and not movement_detected:
                    current_tension -= 0.1
                    target_tensions[i] = current_tension
                    self.servo_controller.async_set_tendons_mm(target_tensions)

                    # Monitor position
                    current_pos = self.get_marker_position()
                    current_servo_pos = self.servo_controller.get_tendons_mm()

                    if current_pos is None:
                        print("\nLost marker tracking! Stopping...")
                        break

                    # Calculate and display movement and servo positions
                    movement = np.linalg.norm(current_pos - initial_pos)

                    # Clear line and print status
                    print("\r" + " " * 100, end="")
                    print(f"\rMovement: {movement:6.2f}mm | ", end="")
                    print("Servo positions: ", end="")
                    for sid, pos in zip(self.config.servo_ids, current_servo_pos):
                        print(f"{sid}:{pos:6.2f}mm ", end="")
                    print("", end="", flush=True)

                    if movement > 1.0:
                        print(f"\nSignificant movement detected!")
                        movement_detected = True
                        break

                    time.sleep(0.05)

            except KeyboardInterrupt:
                print("\nTest interrupted by user")

            # Return to zero
            print("\nReturning to zero position...")
            self.servo_controller.async_set_tendons_mm(
                np.zeros(len(self.config.servo_ids), dtype=np.float64)
            )
            time.sleep(1)

            input("Press Enter to continue to next tendon (or Ctrl+C to stop)...")

        print("\nAll tendon directions checked!")
        return True

    def calibrate(self) -> List[float]:
        """
        Run the main auto-tensioning calibration procedure.

        Returns:
            List of calibrated zero positions in mm
        """
        print("\nStarting auto-tensioning procedure...")
        print(f"Loosen amounts: {self.config.loosen_amount_mm}")
        print(f"Movement thresholds: {self.config.movement_threshold_mm}")
        print(f"Movement countbacks: {self.config.movement_countback_mm}")

        # Verify loosen directions if enabled
        if self.config.verify_loosen_directions:
            print("\n" + "="*60)
            print("Verifying tendon loosen directions...")
            print("="*60)
            if not self.verify_loosen_directions():
                raise Exception("Direction verification failed! Aborting calibration.")

        # Optional interactive direction check
        if self.config.enable_direction_check:
            if input("\nWould you like to check tendon directions interactively? (y/n): ").lower() == 'y':
                if not self.check_tendon_directions(debug=True):
                    raise Exception("Direction check aborted.")

        # Step 1: Loosen all tendons
        print("\nLoosening all tendons...")
        loosen_targets = [-self.config.loosen_amount_mm[i] for i in range(len(self.config.servo_ids))]
        self.servo_controller.async_set_tendons_mm(loosen_targets)
        time.sleep(max(self.config.loosen_amount_mm) + self.config.settling_time_s)
        input("Poke robot slightly to eliminate hysteresis. Press Enter to continue...")

        # Step 2: Find zero positions for each tendon
        zero_positions = []
        initial_pos = self.get_marker_position()
        print(f"Initial position: {initial_pos}")
        if initial_pos is None:
            raise Exception("Cannot detect marker!")

        total_servos = len(self.config.servo_ids)
        for i, servo_id in enumerate(self.config.servo_ids):
            print(f"\n{'='*60}")
            print(f"Calibrating servo {i+1}/{total_servos}: ID {servo_id}")
            print(f"{'='*60}")

            # Set all tendons loose except current one
            tendons_target = [-self.config.loosen_amount_mm[j] for j in range(len(self.config.servo_ids))]
            tendons_target[i] = -1.0
            self.servo_controller.async_set_tendons_mm(tendons_target)
            time.sleep(self.config.loosen_amount_mm[i] + self.config.settling_time_s)

            # Gradually tighten until movement detected
            current_tension = 0
            print("Tightening tendon until movement detected...")
            max_tension = self.config.loosen_amount_mm[i] * 2

            while current_tension < max_tension:
                current_tension += self.config.tightening_increment_mm
                tensions = [-self.config.loosen_amount_mm[j] for j in range(len(self.config.servo_ids))]
                tensions[i] = current_tension
                self.servo_controller.async_set_tendons_mm(tensions)
                time.sleep(self.config.tightening_delay_s)

                current_pos = self.get_marker_position()
                if current_pos is None:
                    raise Exception("Lost marker tracking!")

                # Calculate displacement
                displacement = np.linalg.norm(current_pos - initial_pos)
                progress_bar = draw_progress_bar(
                    displacement,
                    self.config.movement_threshold_mm[i],
                    width=30
                )

                # Display progress
                print(f"\rDisplacement: {displacement:6.2f}mm / {self.config.movement_threshold_mm[i]:.2f}mm {progress_bar}",
                      end="", flush=True)

                if displacement > self.config.movement_threshold_mm[i]:
                    zero_pos = self.servo_controller.get_tendons_mm()[i] - self.config.movement_countback_mm[i]
                    zero_positions.append(zero_pos)
                    print(f"\n✓ Zero position found at {zero_positions[-1]:.2f}mm")
                    break

            # Return to loose position
            self.servo_controller.async_set_tendons_mm(loosen_targets)
            time.sleep(self.config.loosen_amount_mm[i] + self.config.settling_time_s)

        # Step 3: Apply final tensions
        print("\n" + "="*60)
        print("Applying final tensions...")
        print("="*60)
        self.servo_controller.async_set_tendons_mm(zero_positions)
        time.sleep(5)

        current_positions = self.servo_controller.get_tendons_mm()
        print(f"Final positions: {current_positions}")

        # Step 4: Interactive adjustment (if enabled)
        if self.config.enable_interactive_adjustment:
            zero_positions = self._interactive_adjustment(current_positions)

        # Set final zero offsets
        print("\nSetting final zero offsets...")
        self.servo_controller.set_zero_offsets_to_current_position()
        print(f"Zero offsets raw: {self.servo_controller.get_tendons_tick_raw()}")

        print("\n" + "="*60)
        print("Auto-tensioning complete!")
        print("="*60)

        return zero_positions

    def _interactive_adjustment(self, current_positions: np.ndarray) -> List[float]:
        """
        Interactive post-calibration tension adjustment.

        Args:
            current_positions: Current tendon positions

        Returns:
            Final adjusted positions
        """
        print("\n" + "="*60)
        print("Post-Calibration Tension Adjustment")
        print("="*60)
        print("You can now fine-tune the tension on all tendons.")
        print("Examples: '+2' to tighten 2mm, '-1.5' to loosen 1.5mm")
        print("Press Enter (blank) when satisfied.\n")

        adjustment_history = []
        positions = current_positions.copy()

        while True:
            try:
                user_input = input("Adjust all tendons (mm): ").strip()

                if user_input == "":
                    print("Adjustments complete.")
                    break

                # Parse adjustment
                adjustment = float(user_input)

                # Apply adjustment
                adjusted_positions = [pos + adjustment for pos in positions]
                self.servo_controller.async_set_tendons_mm(adjusted_positions)
                time.sleep(2)

                # Update positions
                positions = self.servo_controller.get_tendons_mm()
                adjustment_history.append(adjustment)

                # Display update
                print(f"Applied {adjustment:+.2f}mm adjustment")
                print(f"Updated positions: {positions}")
                print(f"Total adjustment so far: {sum(adjustment_history):+.2f}mm\n")

            except ValueError:
                print("Invalid input. Please enter a number (e.g., '+2', '-1.5') or blank to finish.\n")
            except KeyboardInterrupt:
                print("\nAdjustment interrupted.")
                break

        if adjustment_history:
            print(f"\nTotal manual adjustment applied: {sum(adjustment_history):+.2f}mm")

        return positions.tolist()

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'tracker'):
            self.tracker.stop()
        print("Cleanup complete")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.cleanup()
