# TDCR Tendon Calibration Library

A modular, reusable library for automated tendon calibration of tendon-driven continuum robots (TDCRs) using Intel RealSense cameras and marker tracking.

## Overview

This library provides automated tensioning for TDCRs by:
1. Tracking a colored marker on the robot tip using an Intel RealSense depth camera
2. Progressively tightening each tendon while monitoring tip displacement
3. Detecting the "zero tension" point where the tendon begins to bear load
4. Applying calibrated tensions with optional manual fine-tuning

### Key Features

- **Modular Design**: Easy to adapt for any TDCR with Dynamixel servos
- **Configuration-Driven**: All parameters customizable via dataclasses
- **Hardware Agnostic**: Works with any number of tendons/segments
- **Safety Checks**: Built-in verification for servo loosen directions
- **Interactive**: Post-calibration manual adjustment
- **Well-Documented**: Comprehensive examples and API documentation

## Installation

### Prerequisites

1. **Hardware Requirements:**
   - Intel RealSense depth camera (D435, D455, etc.)
   - Dynamixel servos controlling tendons
   - Colored marker on robot tip (default: red/pink)
   - USB connection to servos (U2D2 or similar)

2. **Software Requirements:**
   - Python 3.7+
   - Ubuntu/Linux (recommended) or macOS

### Setup Instructions

1. **Clone this repository:**
   ```bash
   git clone <repository-url>
   cd tdcr-tendon-calibration
   ```

2. **Clone the crl_tdcr_teleop library into lib/:**
   ```bash
   cd lib
   git clone https://github.com/ContinuumRoboticsLab/crl_tdcr_teleop.git
   cd ..
   ```

3. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Install the library (optional):**
   ```bash
   pip install -e .
   ```

## Quick Start

### Basic Usage

```python
from tdcr_calibration import AutoTensioner, RealSenseTracker, CalibrationConfig, RealSenseConfig

# Configure your robot
config = CalibrationConfig(
    servo_ids=[1, 2, 3],
    device_name="/dev/ttyUSB0"
)

# Configure RealSense camera
rs_config = RealSenseConfig()

# Run calibration
tracker = RealSenseTracker(**vars(rs_config))
tracker.start()

tensioner = AutoTensioner(tracker, config)
zero_positions = tensioner.calibrate()

print(f"Calibrated zero positions: {zero_positions}")
```

### Running Examples

Three example scripts are provided:

1. **Basic calibration** (3-tendon robot with defaults):
   ```bash
   python examples/basic_calibration.py
   ```

2. **ftdcr_v4 calibration** (9-tendon robot with custom parameters):
   ```bash
   python examples/ftdcr_v4_calibration.py
   ```

3. **Custom robot** (template to adapt for your robot):
   ```bash
   cp examples/custom_robot_calibration.py examples/my_robot_calibration.py
   # Edit my_robot_calibration.py with your parameters
   python examples/my_robot_calibration.py
   ```

## Configuration

### Robot Configuration (`CalibrationConfig`)

```python
config = CalibrationConfig(
    # Robot hardware
    servo_ids=[1, 2, 3],              # List of Dynamixel IDs
    spool_radii_mm=10.0,              # Spool radius (mm)
    servo_directions=[1, 1, -1],      # +1 or -1 per servo
    device_name="/dev/ttyUSB0",       # Serial port

    # Calibration parameters
    loosen_amount_mm=5.0,             # Loosen before calibration
    movement_threshold_mm=0.5,        # Displacement for tension detection
    movement_countback_mm=0.0,        # Elasticity compensation

    # Safety and interaction
    verify_loosen_directions=True,    # Verify servo directions
    enable_interactive_adjustment=True,  # Allow manual tuning
)
```

### Camera Configuration (`RealSenseConfig`)

```python
rs_config = RealSenseConfig(
    camera_width=640,
    camera_height=480,
    fps=90,

    # HSV color range for marker
    hsv_range=((165, 70, 70), (15, 255, 255)),  # Red marker

    # Camera to base frame transformation
    T_base_cam=np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]),

    # Smoothing parameters
    smooth_alpha_xy=0.5,
    smooth_alpha_z=0.1,
)
```

## Calibration Process

The automated calibration follows these steps:

### 1. Marker Tracking Verification
- Verifies the colored marker is visible
- Shows real-time position for 5 seconds
- User confirms tracking quality

### 2. Servo Direction Verification
- Loosens each tendon individually
- Checks if tip moves (indicates wrong direction)
- Prevents misconfiguration

### 3. Progressive Tensioning
For each tendon:
- All tendons loosened
- Current tendon gradually tightened
- Monitors tip displacement
- Detects tension engagement threshold
- Saves zero position

### 4. Apply Final Tensions
- Sets all tendons to calibrated positions
- Robot assumes neutral configuration

### 5. Interactive Adjustment (Optional)
- User can fine-tune all tensions
- Applies uniform adjustments (e.g., "+2mm", "-1.5mm")
- Updates final zero offsets

## Hardware Setup

### Marker Setup

1. **Marker Color**: Default is red/pink. Adjust `hsv_range` for other colors:
   - Red: `((165, 70, 70), (15, 255, 255))`
   - Green: `((40, 40, 40), (80, 255, 255))`
   - Blue: `((100, 40, 40), (130, 255, 255))`

2. **Marker Size**: Should be clearly visible at your camera distance
3. **Marker Position**: Attach to robot tip, ensure unobstructed view

### Camera Setup

1. **Mounting**: Stable mount viewing robot workspace
2. **Distance**: Close enough for depth accuracy (~0.3-1.5m typical)
3. **Lighting**: Consistent lighting for color detection
4. **Calibration**: Measure `T_base_cam` transformation matrix for your mounting

### Dynamixel Setup

1. **Servo IDs**: Configure unique IDs for each servo
2. **Baudrate**: Default 3000000 (configured in MultiTendonSyncRW)
3. **Port**: Note your USB serial port (e.g., `/dev/ttyUSB0`)
4. **Power**: Ensure adequate power supply
5. **Spool Radius**: Measure actual spool radius accurately

## Troubleshooting

### Common Issues

**"Cannot detect marker!"**
- Check marker is visible to camera
- Adjust HSV color range for your marker
- Verify lighting conditions
- Check camera view with RealSense Viewer

**"Direction verification failed!"**
- Review `servo_directions` configuration
- Flip direction (+1 → -1 or vice versa) for failed servos
- Verify tendon routing is correct

**"Cannot import crl_tdcr_teleop"**
- Ensure crl_tdcr_teleop is cloned in `lib/` directory
- Check path: `lib/crl_tdcr_teleop/`
- Verify git clone was successful

**"Lost marker tracking!"**
- Marker moved out of camera view
- Lighting changed during calibration
- Reflections interfering with detection
- Try adjusting camera position or smoothing parameters

**Servos not responding**
- Check USB connection to servo controller
- Verify device name (use `ls /dev/tty*`)
- Check servo power supply
- Test servos with Dynamixel Wizard first

### Parameter Tuning

**If tip doesn't move enough during calibration:**
- Increase `loosen_amount_mm` (more slack)
- Decrease `movement_threshold_mm` (more sensitive)

**If calibration is too sensitive:**
- Decrease `loosen_amount_mm`
- Increase `movement_threshold_mm`
- Increase `smooth_alpha_xy` and `smooth_alpha_z` (more filtering)

**If robot is too loose/tight after calibration:**
- Use interactive adjustment to fine-tune
- Adjust `movement_countback_mm` for elasticity compensation

## API Reference

### AutoTensioner

Main calibration class.

**Methods:**
- `calibrate()` - Run full calibration procedure
- `verify_marker_tracking()` - Check marker visibility
- `verify_loosen_directions()` - Verify servo loosen directions are correct
- `check_tendon_directions()` - Interactive direction testing
- `get_marker_position()` - Get current marker position in mm
- `cleanup()` - Clean up resources

### RealSenseTracker

Marker tracking interface.

**Methods:**
- `start()` - Start camera
- `stop()` - Stop camera
- `get_position(in_base_frame=True)` - Get 3D position in meters

### Configuration Classes

- `CalibrationConfig` - Robot and calibration parameters
- `RealSenseConfig` - Camera configuration
- `MotionRecordingConfig` - Optional motion recording parameters

## Advanced Features

### Per-Servo Parameters

Different parameters for each tendon:

```python
config = CalibrationConfig(
    servo_ids=[1, 2, 3],
    loosen_amount_mm=[5.0, 7.0, 6.0],  # Different per tendon
    movement_threshold_mm=[0.5, 1.0, 0.8],
    movement_countback_mm=[0.0, 0.5, 0.2],
)
```

### Custom Camera Transformations

Calibrate camera-to-base transformation:

```python
import numpy as np

# Example: Camera rotated 30° and translated
T_base_cam = np.array([
    [1.0, 0.0, 0.0, 0.05],  # X translation
    [0.0, 0.866, -0.5, 0.10],  # Y translation, rotation
    [0.0, 0.5, 0.866, 0.15],  # Z translation, rotation
    [0.0, 0.0, 0.0, 1.0]
])

rs_config = RealSenseConfig(T_base_cam=T_base_cam)
```
