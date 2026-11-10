# System status and limitations

## Included
- Original `ddqn.py` and original `kuka_dynamics.py`.
- Separate drawing-estimate version `kuka_dynamics_drawing_estimates.py`.
- Existing Stable-Baselines3 DDQN checkpoint ZIP.
- Existing YOLO XYZ Python detector.
- C++17 PCA9685 I2C driver, manual test, and line-protocol daemon.
- Python dry-run integration scaffold and configuration template.

## Not included / not yet verified
- YOLO `best.pt` weights were not available in current files.
- `H_camera_to_robot.npy` calibration was not available and must not be invented.
- Table height, per-class grasp offsets, tray XYZ, servo directions/pulse ranges are unknown.
- The drawing does not uniquely determine joint-axis distances, DH parameters, wrist axes, or TCP.
- Current Kinematics only calculates position from first 3 joint angles; wrist orientation is not modeled.
- No servo encoder feedback exists in the supplied setup; DDQN control in simulation is not closed-loop on real joints.
- This project is not yet safe/verified for autonomous physical pick-and-place.

## Important
Do not enable physical movement merely by changing flags. First calibrate all six servos and verify joint limits,
coordinate frames, FK/IK, grasping, and emergency stop behavior. Use external servo power; share ground with Pi.
