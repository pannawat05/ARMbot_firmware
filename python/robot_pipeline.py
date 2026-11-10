#!/usr/bin/env python3
"""
Integration scaffold: camera -> YOLO XYZ -> IK goal -> trained DDQN actions -> C++ PCA9685 daemon.

SAFETY:
- Dry-run is the default.
- The existing checkpoint was trained with the original kuka_dynamics.py.
- Its simulated joint state is open-loop (there are no joint encoders in this project),
  so this program MUST NOT be treated as ready for physical pick-and-place.
- Real motion is intentionally blocked until motion_enabled and servo calibration are
  explicitly configured, but those flags alone do not replace calibration/validation.
- The current legacy Kinematics is a 3-DOF positional approximation; wrist axes/TCP are not
  modeled. This pipeline is a integration scaffold, not verified autonomous manipulation.
"""
import argparse, json, math, subprocess, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
import kuka_dynamics as kd
from yolo_xyz_raspberry_pi5 import YoloXYZDetector

def load_config(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT/"calibration"/"config.json"))
    ap.add_argument("--dry-run", action="store_true", help="Never command real servos (default)")
    ap.add_argument("--once", action="store_true", help="Process one camera frame and exit")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    cfg = load_config(args.config)
    cam = cfg["camera"]; geom = cfg["geometry"]; hw = cfg["hardware"]; dd = cfg["ddqn"]

    if not args.dry_run:
        if not hw.get("motion_enabled", False) or not hw.get("servo_calibration_confirmed", False):
            raise SystemExit("Blocked: set hardware.motion_enabled=true and servo_calibration_confirmed=true only after hardware calibration.")
        raise SystemExit("Physical pick-place is not enabled by this scaffold: joint feedback, wrist/TCP Kinematics, and verified grasp/place sequencing are still required.")

    weights = ROOT / cam["weights_path"]
    Hpath = ROOT / cam["homography_path"]
    if not weights.exists():
        raise SystemExit(f"Missing YOLO weights: {weights}. Add your trained best.pt.")
    if not Hpath.exists():
        raise SystemExit(f"Missing calibration homography: {Hpath}. Create it from measured table points.")
    if geom["table_z_mm"] is None or geom["tray_xyz_mm"] is None:
        raise SystemExit("Set geometry.table_z_mm and geometry.tray_xyz_mm in calibration/config.json using measured values.")
    if any(v is None for v in geom["grasp_z_offsets_mm"].values()):
        raise SystemExit("Set a measured grasp_z_offsets_mm for every target class before XYZ is complete.")

    from stable_baselines3 import DQN
    checkpoint = ROOT / dd["checkpoint"]
    model = DQN.load(str(checkpoint), device="cpu")
    detector = YoloXYZDetector(
        weights_path=str(weights), homography_path=str(Hpath),
        source=cam["source"], confidence=cam["confidence"],
        table_z_mm=geom["table_z_mm"],
        grasp_z_offsets=geom["grasp_z_offsets_mm"],
        pixel_point_mode=cam["pixel_point_mode"]
    )
    cap = detector.open_capture()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera frame read failed.")
            result = detector.detect_frame(frame)
            target = result["target"]
            if target is None:
                print("No object detected.")
            elif not result["xyz_complete"]:
                print("Detection has no complete XYZ; no action taken:", target)
            else:
                xyz = np.asarray(target["xyz_mm"], dtype=np.float32)
                q_goal = kd.solve_ik(xyz)
                if q_goal is None:
                    print(f"Target outside legacy model workspace: XYZ={xyz.tolist()}")
                else:
                    # State must match training: [q(6), q_goal(6), previous_velocity(6)].
                    # This is a dry-run simulation only; no joint feedback is available.
                    q = np.zeros(6, dtype=np.float32)
                    prev_v = np.zeros(6, dtype=np.float32)
                    for step in range(int(dd["max_steps"])):
                        obs = kd.make_observation(q, q_goal, prev_v)
                        action, _ = model.predict(obs, deterministic=True)
                        action = int(np.asarray(action).reshape(-1)[0])
                        q, prev_v, reward, reached, dist = kd.step_dynamics(
                            q, q_goal, prev_v, action
                        )
                        if reached:
                            break
                    print(json.dumps({
                        "mode": "DRY_RUN_ONLY",
                        "class": target["class_name"],
                        "xyz_mm": target["xyz_mm"],
                        "ik_goal_rad": q_goal.tolist(),
                        "ddqn_simulated_q_rad": q.tolist(),
                        "steps": step+1,
                        "joint_goal_reached": bool(reached),
                        "note": "No physical servo command was sent. This does not perform gripper pick/place."
                    }, indent=2))
            if args.show:
                import cv2
                cv2.imshow("Robot pipeline dry-run", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"): break
            if args.once: break
    finally:
        cap.release()
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
