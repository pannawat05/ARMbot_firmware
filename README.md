# YOLO XYZ + DDQN + Raspberry Pi 5 + PCA9685 (C++/Python)

This ZIP combines the available project files in one repository. It includes a **dry-run integration scaffold**,
not a claim that autonomous physical pick-and-place is already calibrated.

## Hardware/software architecture
ESP32-CAM/OV3660 stream -> Raspberry Pi 5 Python YOLO XYZ -> legacy IK/DDQN dry-run -> (future) C++ PCA9685 daemon -> six servos.

The C++ controller uses Linux `i2c-dev`, not pigpio. The PCA9685 generates servo PWM in hardware; the Pi communicates over I2C.
The `servo_daemon` command protocol is `MOVE <six angles in degrees>`, `OFF`, and `QUIT`, but the physical bridge is intentionally
not connected by default.

## Install on Raspberry Pi OS
```bash
sudo apt update
sudo apt install -y build-essential cmake i2c-tools python3-venv
sudo raspi-config   # enable I2C
sudo reboot
```
Check the board address:
```bash
sudo i2cdetect -y 1
```
Build C++:
```bash
cd cpp
cmake -S . -B build
cmake --build build -j
```
Python:
```bash
cd ..
python3 -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
```

## Required assets before camera dry-run
1. Copy YOLO weights to `models/best.pt`.
2. Create real `calibration/H_camera_to_robot.npy` from measured points at 640x480.
3. Edit `calibration/config.json`: actual stream URL, measured `table_z_mm`, class grasp offsets, tray XYZ.
4. Check model file and Python dependencies.

Run only dry-run:
```bash
python python/robot_pipeline.py --dry-run --once
```
To preview:
```bash
python python/robot_pipeline.py --dry-run --show
```

## Safety
- Physical servo motion is not enabled in the Python pipeline.
- Do not connect or run servo commands with uncalibrated limits.
- Use an external servo power supply sized for all six servos, common ground, and a safe mechanical test setup.
- The included DH/drawing estimates are provisional. The current original Kinematics is a 3-DOF positional approximation, not a validated six-axis model.
- The DDQN checkpoint was trained against the original state/dynamics assumptions. A changed robot geometry may require evaluation/retraining.
- A single homography only maps the table plane to XY; it does not measure object height/depth.
