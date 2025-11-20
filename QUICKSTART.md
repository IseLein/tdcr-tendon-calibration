# Quick Start Guide

Get your TDCR calibrated in 5 minutes!

## Step 1: Clone Dependencies

```bash
cd lib
git clone https://github.com/ContinuumRoboticsLab/crl_tdcr_teleop.git
cd ..
```

## Step 2: Install Requirements

```bash
pip install -r requirements.txt
```

## Step 3: Choose Your Example

### For a Simple 3-Tendon Robot:
Edit `examples/basic_calibration.py` with your servo IDs and run:
```bash
python examples/basic_calibration.py
```

### For ftdcr_v4 (9 tendons):
Edit the device name in `examples/ftdcr_v4_calibration.py` and run:
```bash
python examples/ftdcr_v4_calibration.py
```

### For Your Custom Robot:
Copy and edit the template:
```bash
cp examples/custom_robot_calibration.py examples/my_robot.py
# Edit my_robot.py with your parameters
python examples/my_robot.py
```

## Step 4: Prepare Hardware

1. **Attach colored marker** to robot tip (default: red/pink)
2. **Position RealSense camera** viewing the robot workspace
3. **Connect Dynamixel servos** via USB
4. **Power on** servos

## Step 5: Run Calibration

The script will guide you through:
1. ✓ Marker tracking verification (5 seconds)
2. ✓ Servo direction verification (loosen check)
3. ✓ Progressive tensioning (finds zero points)
4. ✓ Manual fine-tuning (optional)

## Key Parameters to Adjust

```python
# In your calibration script:

# Your hardware
servo_ids=[1, 2, 3]          # Your Dynamixel IDs
device_name="/dev/ttyUSB0"    # Check with: ls /dev/tty*

# Calibration sensitivity
loosen_amount_mm=5.0          # Increase if tip doesn't move enough
movement_threshold_mm=0.5     # Decrease for more sensitivity

# Marker color (HSV range)
hsv_range=((165, 70, 70), (15, 255, 255))  # Red (default)
# hsv_range=((40, 40, 40), (80, 255, 255))  # Green
# hsv_range=((100, 40, 40), (130, 255, 255))  # Blue
```

## Troubleshooting

**Can't detect marker?**
- Check camera view with RealSense Viewer
- Adjust HSV color range
- Improve lighting

**Direction verification fails?**
- Flip servo direction: `[1, 1, -1]` → `[1, 1, 1]`
- Check tendon routing

**Import error?**
- Ensure `lib/crl_tdcr_teleop/` exists
- Run: `ls lib/crl_tdcr_teleop`

## Next Steps

After calibration:
- Zero positions are saved to servos
- Robot maintains calibrated state
- Use positions in your control code

See `README.md` for detailed documentation.
