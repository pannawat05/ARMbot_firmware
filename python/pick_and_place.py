#!/usr/bin/env python3
"""Pick-and-place for the 6-servo arm on Raspberry Pi 5 + PCA9685.

Pipeline:
  ESP32-CAM JPEG stream over USB serial (/dev/ttyUSB0), or a USB webcam with --camera usb
  -> YOLO (models/best.pt) finds the named object
  -> table homography turns its pixel into robot XY; Z comes from table height + per-class grasp offset
  -> ArUco marker gives the tray position the same way
  -> MCCD inverse kinematics on an IKPy chain gives joint angles
  -> the existing DDQN checkpoint plans the joint-space trajectory between poses
  -> PCA9685 drives the servos (dry run unless --execute)

Usage (run with the .venv-cpu interpreter):
  python pick_and_place.py bananapi                     # dry run: camera + plan, no servo output
  python pick_and_place.py bananapi --show              # same, with an annotated camera window
  python pick_and_place.py bananapi --object-xyz 180 0 10 --tray-xyz 120 150 30   # no camera
  python pick_and_place.py bananapi --execute           # move the real arm (after calibration)
  python pick_and_place.py any --auto --execute         # keep picking any known object until none is left
  python pick_and_place.py bananapi --preview           # live camera: detections + ArUco ids, no motion
  python pick_and_place.py --aruco-preview              # live camera: ArUco ids only, no YOLO, no motion
  python pick_and_place.py --calibrate-homography       # build the table homography from ArUco markers

Safety:
- Hobby servos give no position feedback. The arm must be resting at the home pose
  before --execute, because the first command jumps straight to it.
- Every servo angle of the whole plan is checked against the per-joint limits before
  anything moves; one violation aborts the run.
- Ctrl+C stops sending commands. The PCA9685 keeps holding the last pose, so the arm
  does not drop; press ALL OFF in servo_control_app.py or cut servo power afterwards.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import kuka_dynamics as kd  # noqa: E402  DDQN state/dynamics the checkpoint was trained on
from servo_control_app import SERVO_TYPES, DryRunPca9685, Pca9685, angle_to_us  # noqa: E402

DEFAULT_CONFIG = ROOT / "calibration" / "pick_place_config.json"
NUM_JOINTS = 6
USB_STALE_S = 0.15  # a longer gap between reads leaves an old frame in the USB camera buffer
OPEN_TIMEOUT_S = 3.0  # a working ESP32-CAM sends its first frame about 1.7 s after the reset
LINEAR_STEP_RAD = math.radians(2.0)  # max joint change per command on straight segments
DDQN_CLOSE_RAD = math.radians(15.0)  # accept a DDQN path that ends this close to the goal


# --------------------------------------------------------------------------- config

def load_config(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def rel(path):
    p = Path(path)
    return p if p.is_absolute() else ROOT / p


# --------------------------------------------------------------------------- camera

class SerialCamera:
    """ESP32-CAM sending raw back-to-back JPEGs over USB serial (see test_camera_yolo.py)."""

    def __init__(self, cfg, attempts=3):
        from test_camera_yolo import JpegStream, open_serial

        self.size = (cfg["width"], cfg["height"])
        self.timeout_s = cfg["frame_timeout_s"]
        self.mirror = cfg["mirror"]
        # A plain open often leaves the ESP32 silent after its boot text; a reset pulse starts it reliably.
        for attempt in range(1, attempts + 1):
            self.ser = open_serial(cfg["port"], cfg["baud"], reset=True)
            self.stream = JpegStream(self.ser)
            try:
                self.read(OPEN_TIMEOUT_S)
                return
            except RuntimeError:
                self.ser.close()
                print(f"  No camera frames (try {attempt}/{attempts}).")
        raise SystemExit("Camera sent no frames; press RESET on the ESP32-CAM and check the port.")

    def read(self, timeout_s=None):
        import cv2

        deadline = time.monotonic() + (timeout_s or self.timeout_s)
        while time.monotonic() < deadline:
            jpeg = self.stream.read_latest()
            if jpeg is None:
                continue
            img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                # The homography is calibrated at this resolution.
                if (img.shape[1], img.shape[0]) != self.size:
                    img = cv2.resize(img, self.size)
                return cv2.flip(img, 1) if self.mirror else img
        raise RuntimeError("No JPEG frame from the ESP32-CAM; check the port, baud and RESET button.")

    def close(self):
        self.ser.close()


class UsbCamera:
    """USB webcam found by its name in /sys/class/video4linux, e.g. a Logitech C270."""

    def __init__(self, cfg):
        import cv2

        self.size = (cfg["width"], cfg["height"])
        self.timeout_s = cfg["frame_timeout_s"]
        self.mirror = cfg["usb_mirror"]
        self.last_read = 0.0
        device = self.find_device(cfg["usb_name"])
        self.cap = cv2.VideoCapture(int(device.removeprefix("/dev/video")), cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open {device}; check that no other program is using it.")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep frames fresh while YOLO is busy
        try:
            self.read()
        except RuntimeError as e:
            self.close()
            raise SystemExit(str(e))
        print(f"USB camera on {device}.")

    @staticmethod
    def find_device(name):
        """First capture node whose name contains name; the lowest index is the video stream."""
        nodes = sorted(Path("/sys/class/video4linux").glob("video*"), key=lambda n: int(n.name[5:]))
        seen = [(n.name, (n / "name").read_text().strip()) for n in nodes]
        for node, label in seen:
            if name.lower() in label.lower():
                return f"/dev/{node}"
        raise SystemExit(f"No USB camera named '{name}'. Found: {sorted({label for _, label in seen})}")

    def read(self):
        import cv2

        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if time.monotonic() - self.last_read > USB_STALE_S:
                self.cap.grab()  # drop the old frame left in the buffer during a pause
            ok, img = self.cap.read()
            self.last_read = time.monotonic()
            if ok and img is not None:
                if (img.shape[1], img.shape[0]) != self.size:
                    img = cv2.resize(img, self.size)
                return cv2.flip(img, 1) if self.mirror else img
        raise RuntimeError("No frame from the USB camera; check the cable.")

    def close(self):
        self.cap.release()


def open_camera(cfg):
    return UsbCamera(cfg) if cfg["source"] == "usb" else SerialCamera(cfg)


# --------------------------------------------------------------------------- vision

def aruco_detector(dict_name):
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def find_markers(detector, frame):
    """Return {marker_id: (u, v)} marker centres in pixels."""
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return {}
    return {int(i): tuple(c.reshape(4, 2).mean(axis=0)) for i, c in zip(ids.flatten(), corners)}


def find_object(model, frame, class_name, ycfg):
    """Best detection of class_name ("any" = every class) as (u, v, confidence, bbox, name) or None."""
    result = model.predict(frame, imgsz=ycfg["imgsz"], conf=ycfg["confidence"],
                           device="cpu", verbose=False)[0]
    best = None
    for box in result.boxes:
        name = result.names[int(box.cls[0])]
        if class_name != "any" and name != class_name:
            continue
        conf = float(box.conf[0])
        if best is None or conf > best[2]:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            v = y2 if ycfg["pixel_point"] == "bottom_center" else (y1 + y2) / 2
            best = ((x1 + x2) / 2, v, conf, (x1, y1, x2, y2), name)
    return best


def load_homography(path):
    if not path.exists():
        raise SystemExit(f"Missing {path}. Run: python pick_and_place.py --calibrate-homography")
    H = np.load(path)
    if H.shape != (3, 3) or not np.isfinite(H).all() or abs(np.linalg.det(H)) < 1e-12:
        raise SystemExit(f"{path} is not a valid 3x3 homography.")
    return H


def pixel_to_xy(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return p[0] / p[2], p[1] / p[2]


class TargetNotFound(Exception):
    """The object or the tray marker was not seen in enough frames."""


def locate_targets(cfg, class_name, show):
    """Average several camera frames; return (object_xyz, tray_xyz, class_name) in mm."""
    import cv2
    from ultralytics import YOLO

    H = load_homography(rel(cfg["table"]["homography"]))
    model = YOLO(str(rel(cfg["yolo"]["weights"])))
    if class_name != "any" and class_name not in model.names.values():
        raise SystemExit(f"'{class_name}' is not a YOLO class. Known: {', '.join(model.names.values())}")
    aruco = aruco_detector(cfg["aruco"]["dictionary"])
    tray_id = cfg["aruco"]["tray_marker_id"]

    cam = open_camera(cfg["camera"])
    obj_px, tray_px, names = [], [], []
    try:
        for _ in range(cfg["camera"]["frames_to_average"]):
            frame = cam.read()
            obj = find_object(model, frame, class_name, cfg["yolo"])
            marker = find_markers(aruco, frame).get(tray_id)
            if obj:
                obj_px.append(obj[:2])
                names.append(obj[4])
            if marker:
                tray_px.append(marker)
            if show:
                if obj:
                    x1, y1, x2, y2 = (int(c) for c in obj[3])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{obj[4]} {obj[2]:.2f}", (x1, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                if marker:
                    cv2.circle(frame, (int(marker[0]), int(marker[1])), 6, (255, 0, 255), 2)
                cv2.imshow("pick_and_place (q to close)", frame)
                cv2.waitKey(1)
    finally:
        cam.close()

    n = cfg["camera"]["frames_to_average"]
    if len(obj_px) < n // 2 + 1:
        raise TargetNotFound(f"'{class_name}' seen in only {len(obj_px)}/{n} frames.")
    if len(tray_px) < n // 2 + 1:
        raise TargetNotFound(f"Tray ArUco marker id {tray_id} seen in only {len(tray_px)}/{n} frames.")

    t = cfg["table"]
    found = max(set(names), key=names.count)
    ox, oy = pixel_to_xy(H, *np.median(obj_px, axis=0))
    oz = t["table_z_mm"] + t["grasp_z_offset_mm"].get(found, 0.0)
    tx, ty = pixel_to_xy(H, *np.median(tray_px, axis=0))
    dx, dy = cfg["aruco"]["tray_offset_xy_mm"]
    return np.array([ox, oy, oz]), np.array([tx + dx, ty + dy, cfg["aruco"]["place_z_mm"]]), found


def preview(cfg, class_name):
    """Live camera window: YOLO boxes, ArUco ids and (if calibrated) robot XY. q quits.

    class_name None leaves YOLO out and shows ArUco only.
    """
    import cv2

    hpath = rel(cfg["table"]["homography"])
    H = load_homography(hpath) if hpath.exists() else None
    model = None
    if class_name:
        from ultralytics import YOLO

        model = YOLO(str(rel(cfg["yolo"]["weights"])))
    aruco = aruco_detector(cfg["aruco"]["dictionary"])
    ycfg = cfg["yolo"]
    cam = open_camera(cfg["camera"])
    print("Preview: q in the window to quit." + ("" if H is not None else
          " No homography yet, so no robot XY is shown."))
    try:
        while True:
            frame = cam.read()
            result = model.predict(frame, imgsz=ycfg["imgsz"], conf=ycfg["confidence"],
                                   device="cpu", verbose=False)[0] if model else None
            for box in result.boxes if model else ():
                name = result.names[int(box.cls[0])]
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                color = (0, 255, 0) if name == class_name else (0, 200, 255)
                label = f"{name} {float(box.conf[0]):.2f}"
                if H is not None:
                    v = y2 if ycfg["pixel_point"] == "bottom_center" else (y1 + y2) / 2
                    label += " X%.0f Y%.0f" % pixel_to_xy(H, (x1 + x2) / 2, v)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            markers = find_markers(aruco, frame)
            if not model:
                cv2.putText(frame, f"ArUco: {sorted(markers) or 'none'}", (8, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            for mid, (u, v) in markers.items():
                tag = "tray" if mid == cfg["aruco"]["tray_marker_id"] else f"id {mid}"
                if H is not None:
                    tag += " X%.0f Y%.0f" % pixel_to_xy(H, u, v)
                cv2.circle(frame, (int(u), int(v)), 6, (255, 0, 255), 2)
                cv2.putText(frame, tag, (int(u) + 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
            cv2.imshow("pick_and_place preview (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()


def calibrate_homography(cfg):
    """Fit pixel -> robot XY from ArUco markers placed flat at known table positions."""
    import cv2

    known = {int(k): v for k, v in cfg["aruco"]["calibration_markers_xy_mm"].items()}
    aruco = aruco_detector(cfg["aruco"]["dictionary"])
    cam = open_camera(cfg["camera"])
    seen = {}
    try:
        for _ in range(cfg["camera"]["frames_to_average"]):
            for mid, uv in find_markers(aruco, cam.read()).items():
                if mid in known:
                    seen.setdefault(mid, []).append(uv)
    finally:
        cam.close()
    if len(seen) < 4:
        raise SystemExit(f"Need 4+ calibration markers in view, saw ids {sorted(seen)} of {sorted(known)}.")

    ids = sorted(seen)
    px = np.array([np.median(seen[i], axis=0) for i in ids], dtype=np.float64)
    xy = np.array([known[i] for i in ids], dtype=np.float64)
    H, _ = cv2.findHomography(px, xy)
    if H is None:
        raise SystemExit("Homography fit failed; spread the markers out and avoid putting them in a line.")
    err = [np.hypot(*(np.array(pixel_to_xy(H, *p)) - q)) for p, q in zip(px, xy)]
    out = rel(cfg["table"]["homography"])
    np.save(out, H)
    print(f"Saved {out} from markers {ids}; max reprojection error {max(err):.1f} mm")


# --------------------------------------------------------------------------- kinematics

class ArmIK:
    """IKPy chain of the arm plus a Modified Cyclic Coordinate Descent (MCCD) solver.

    Plain CCD rotates one joint at a time (tip to base) to swing the end point toward
    the target. The modifications here: joint limits are enforced on every update,
    each update is damped (step cap that shrinks over the sweeps), the base yaw is
    seeded analytically, several seeds are tried, and the best result is polished
    with IKPy's own optimiser.
    """

    def __init__(self, arm):
        from ikpy.chain import Chain
        from ikpy.link import OriginLink, URDFLink

        lo, hi = kd.JOINT_LOW.astype(float), kd.JOINT_HIGH.astype(float)
        pitch_axis = [0, -1, 0]  # positive angle lifts the link (matches kuka_dynamics)
        self.wrist_mode = arm["wrist_mode"]
        self.wrist_fixed = math.radians(arm["wrist_fixed_deg"])
        self.tool = arm["tool_length_mm"]
        self.chain = Chain(name="arm", links=[
            OriginLink(),
            URDFLink("base", [0, 0, 0], [0, 0, 0], rotation=[0, 0, 1], bounds=(lo[0], hi[0])),
            URDFLink("shoulder", [0, 0, arm["shoulder_height_mm"]], [0, 0, 0],
                     rotation=pitch_axis, bounds=(lo[1], hi[1])),
            URDFLink("elbow", [arm["upper_arm_mm"], 0, 0], [0, 0, 0],
                     rotation=pitch_axis, bounds=(lo[2], hi[2])),
            URDFLink("wrist_pitch", [arm["forearm_mm"], 0, 0], [0, 0, 0],
                     rotation=pitch_axis, bounds=(lo[3], hi[3])),
            URDFLink("tool", [self.tool, 0, 0], [0, 0, 0], joint_type="fixed"),
        ], active_links_mask=[False, True, True, True, False, False])
        self.lo, self.hi = lo, hi
        self.axes = [None, [0, 0, 1], pitch_axis, pitch_axis, pitch_axis, None]

    def _full(self, q):
        """DDQN joint vector (6) -> IKPy joint vector (one per link)."""
        return [0.0, q[0], q[1], q[2], q[3], 0.0]

    def frames(self, q):
        return self.chain.forward_kinematics(self._full(q), full_kinematics=True)

    def tool_xyz(self, q):
        return self.frames(q)[-1][:3, 3]

    def _wrist_for(self, q):
        if self.wrist_mode == "vertical":
            return -math.pi / 2 - (q[1] + q[2])  # keeps the tool pointing straight down
        return self.wrist_fixed

    def _ccd(self, target, q, iters=150, tol=0.5):
        # In vertical mode the wrist pitch follows the arm, so aim the wrist centre
        # straight above the target; in fixed mode aim the tool tip itself.
        vertical = self.wrist_mode == "vertical"
        aim = target + np.array([0, 0, self.tool]) if vertical else target
        end_index = 4 if vertical else 5
        q = q.copy()
        for sweep in range(iters):
            cap = 0.35 * (1.0 - sweep / iters) + 0.02  # damping
            for j in (2, 1, 0):  # elbow, shoulder, base (tip to base)
                q[3] = self._wrist_for(q)
                fr = self.frames(q)
                link = j + 1
                pivot = fr[link][:3, 3]
                axis = fr[link][:3, :3] @ np.array(self.axes[link], dtype=float)
                u = fr[end_index][:3, 3] - pivot
                v = aim - pivot
                u -= axis * axis.dot(u)
                v -= axis * axis.dot(v)
                if np.linalg.norm(u) < 1e-6 or np.linalg.norm(v) < 1e-6:
                    continue
                d = math.atan2(axis.dot(np.cross(u, v)), u.dot(v))
                q[j] = np.clip(q[j] + np.clip(d, -cap, cap), self.lo[j], self.hi[j])
            q[3] = self._wrist_for(q)
            fr = self.frames(q)
            if np.linalg.norm(fr[end_index][:3, 3] - aim) < tol:
                break
        return q, float(np.linalg.norm(self.tool_xyz(q) - target))

    def solve(self, target, q_seed):
        target = np.asarray(target, dtype=float)
        yaw = float(np.clip(math.atan2(target[1], target[0]), self.lo[0], self.hi[0]))
        seeds = [np.asarray(q_seed, dtype=float)]
        for sh, el in ((0.6, -1.2), (1.0, -1.4), (0.3, -0.6)):
            seeds.append(np.array([yaw, sh, el, 0.0, 0.0, 0.0]))
        best = None
        for s in seeds:
            s = s.copy()
            s[0] = yaw
            q, err = self._ccd(target, s)
            if best is None or err < best[1]:
                best = (q, err)
        q, err = best

        # Polish with IKPy's optimiser; keep it only if it is better and legal.
        try:
            full = self.chain.inverse_kinematics(target, initial_position=self._full(q))
            q2 = np.array([full[1], full[2], full[3], full[4], 0.0, 0.0])
            q2[3] = self._wrist_for(q2)
            err2 = float(np.linalg.norm(self.tool_xyz(q2) - target))
            if err2 < err and np.all(q2[:4] >= self.lo[:4]) and np.all(q2[:4] <= self.hi[:4]):
                q, err = q2, err2
        except Exception:
            pass
        q[4] = q[5] = 0.0
        return q, err


# --------------------------------------------------------------------------- planning

def ddqn_path(model, ik, q_start, q_goal, max_steps):
    """Joint-space path from the DDQN policy; finished with a short straight segment.

    Only base/shoulder/elbow are planned by the policy. The wrist pitch is derived
    from them (ArmIK._wrist_for) and wrist roll stays 0, so those joints are held at
    0 in the policy's view to keep its input inside the range it was trained on.
    """
    def policy_view(q):
        out = np.zeros(NUM_JOINTS, dtype=np.float32)
        out[:3] = q[:3]
        return out

    q, goal = policy_view(q_start), policy_view(q_goal)
    v = np.zeros(NUM_JOINTS, dtype=np.float32)
    path = [np.asarray(q_start, float)]
    dists = [float(np.linalg.norm(goal - q))]
    reached = False
    for _ in range(max_steps):
        action, _ = model.predict(kd.make_observation(q, goal, v), deterministic=True)
        q, v, _, reached, dist = kd.step_dynamics(q, goal, v, int(np.asarray(action).reshape(-1)[0]))
        q[3:] = 0.0  # the policy sometimes nudges joints it does not own; undo that
        full = q.astype(float)
        full[3] = ik._wrist_for(full)
        path.append(full)
        dists.append(dist)
        if reached:
            break
    if not reached:
        # The checkpoint tends to get close and then dither. Keep the policy's path up
        # to its closest point and finish straight, unless it never got near.
        best = int(np.argmin(dists))
        if dists[best] > DDQN_CLOSE_RAD:
            print(f"  WARN: DDQN got no closer than {math.degrees(dists[best]):.0f}°; "
                  "using a straight joint-space move instead.")
            return linear_path(q_start, q_goal), False
        print(f"  note: DDQN stopped {math.degrees(dists[best]):.1f}° short; finishing straight.")
        path = path[:best + 1]
    return path + linear_path(path[-1], q_goal)[1:], True


def linear_path(q_start, q_goal):
    q_start, q_goal = np.asarray(q_start, float), np.asarray(q_goal, float)
    n = max(1, int(math.ceil(np.max(np.abs(q_goal - q_start)) / LINEAR_STEP_RAD)))
    return [q_start + (q_goal - q_start) * k / n for k in range(n + 1)]


class Joints:
    """Maps DDQN joint angles (rad) to servo angles and pulses."""

    def __init__(self, jcfg, gcfg):
        self.list = jcfg["list"]
        self.gripper = gcfg
        for j in self.list + [gcfg]:
            if j["type"] not in SERVO_TYPES:
                raise SystemExit(f"Unknown servo type {j['type']}; known: {', '.join(SERVO_TYPES)}")

    def servo_deg(self, q):
        return [j["center_deg"] + j["direction"] * math.degrees(a) for j, a in zip(self.list, q)]

    def joint_rad(self, servo_deg):
        """Servo angles as shown in servo_control_app (base, shoulder, elbow) -> joint rad."""
        q = np.zeros(NUM_JOINTS)
        for i, (j, deg) in enumerate(zip(self.list, servo_deg)):
            q[i] = math.radians((deg - j["center_deg"]) / j["direction"])
        return q

    def check(self, path, label):
        for q in path:
            for j, deg in zip(self.list, self.servo_deg(q)):
                if j["enabled"] and not j["min_deg"] - 1e-6 <= deg <= j["max_deg"] + 1e-6:
                    raise SystemExit(
                        f"ABORT ({label}): {j['name']} needs servo {deg:.1f}° outside "
                        f"{j['min_deg']}–{j['max_deg']}°. Nothing was moved.")


# --------------------------------------------------------------------------- main flow

def build_plan(cfg, ik, ddqn, joints, obj_xyz, tray_xyz):
    arm, up = cfg["arm"], np.array([0, 0, cfg["table"]["approach_height_mm"]])
    home = joints.joint_rad(arm["home_servo_deg"])
    home[3] = ik._wrist_for(home)

    poses = {}
    q = home
    for name, xyz in (("pre_grasp", obj_xyz + up), ("grasp", obj_xyz),
                      ("pre_place", tray_xyz + up), ("place", tray_xyz)):
        q, err = ik.solve(xyz, q)
        reach = np.linalg.norm(ik.tool_xyz(q) - xyz)
        print(f"  IK {name:9s} target {np.round(xyz, 1)} mm -> joints "
              f"{np.round(np.degrees(q[:4]), 1)}° (error {err:.1f} mm)")
        if reach > 5.0:
            raise SystemExit(f"ABORT: {name} {np.round(xyz, 1)} mm is out of reach "
                             f"(best error {reach:.0f} mm). Nothing was moved.")
        poses[name] = q

    g = cfg["gripper"]
    plan = [("open gripper", "gripper", g["open_deg"])]
    for label, a, b, planner in (
        ("home -> above object", home, poses["pre_grasp"], "ddqn"),
        ("descend to object", poses["pre_grasp"], poses["grasp"], "linear"),
        ("grip", None, None, g["grip_deg"]),
        ("lift", poses["grasp"], poses["pre_grasp"], "linear"),
        ("carry -> above tray", poses["pre_grasp"], poses["pre_place"], "ddqn"),
        ("lower into tray", poses["pre_place"], poses["place"], "linear"),
        ("release", None, None, g["open_deg"]),
        ("retreat", poses["place"], poses["pre_place"], "linear"),
        ("return home", poses["pre_place"], home, "ddqn"),
    ):
        if a is None:
            plan.append((label, "gripper", planner))
            continue
        path = ddqn_path(ddqn, ik, a, b, cfg["ddqn"]["max_steps"])[0] if planner == "ddqn" else linear_path(a, b)
        joints.check(path, label)
        plan.append((label, "path", path))
    return home, plan


def execute(cfg, joints, home, plan, pca, dry_run, cut_pwm):
    g = cfg["gripper"]

    def send_pose(q):
        for j, deg in zip(joints.list, joints.servo_deg(q)):
            if j["enabled"]:
                pca.set_pulse_us(j["channel"], angle_to_us(j["type"], deg))

    print("Sending home pose (arm must already be resting there).")
    send_pose(home)
    time.sleep(0.5 if not dry_run else 0)
    for label, kind, data in plan:
        print(f"> {label}")
        if kind == "gripper":
            pca.set_pulse_us(g["channel"], angle_to_us(g["type"], data))
            time.sleep(g["settle_s"] if not dry_run else 0)
            continue
        for q in data[1:]:
            send_pose(q)
            time.sleep(kd.DT if not dry_run else 0)
    if cut_pwm:
        pca.all_off()


def run_cycle(cfg, args, ik, ddqn, joints, pca, dry_run, cut_pwm):
    """Find the object and tray, plan every move, then run it. Raises TargetNotFound."""
    if args.object_xyz is not None and args.tray_xyz is not None:
        obj_xyz, tray_xyz, found = np.array(args.object_xyz), np.array(args.tray_xyz), args.object
    else:
        print("Looking for the object and tray marker on the camera...")
        obj_xyz, tray_xyz, found = locate_targets(cfg, args.object, args.show)
        if args.object_xyz is not None:
            obj_xyz = np.array(args.object_xyz)
        if args.tray_xyz is not None:
            tray_xyz = np.array(args.tray_xyz)
    print(f"Object '{found}' at {np.round(obj_xyz, 1)} mm, tray at {np.round(tray_xyz, 1)} mm")

    # Plan every move before touching the servos.
    print("Planning (MCCD IK + DDQN)...")
    home, plan = build_plan(cfg, ik, ddqn, joints, obj_xyz, tray_xyz)
    n_cmds = sum(len(d) - 1 for _, k, d in plan if k == "path")
    print(f"Plan OK: {len(plan)} steps, {n_cmds} pose commands, all servo angles within limits.")
    execute(cfg, joints, home, plan, pca, dry_run, cut_pwm)


def main():
    ap = argparse.ArgumentParser(description="YOLO + ArUco + MCCD IK + DDQN pick-and-place")
    ap.add_argument("object", nargs="?", help="YOLO class name to pick, e.g. bananapi, or 'any'")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--execute", action="store_true", help="drive the real servos (default: dry run)")
    ap.add_argument("--show", action="store_true", help="show the annotated camera frames")
    ap.add_argument("--object-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="skip the camera for the object position (mm)")
    ap.add_argument("--tray-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="skip the camera for the tray position (mm)")
    ap.add_argument("--auto", action="store_true",
                    help="keep picking: look, pick, place, repeat until nothing is found")
    ap.add_argument("--max-cycles", type=int, default=0, help="--auto: stop after N picks (0 = no limit)")
    ap.add_argument("--max-misses", type=int, default=5,
                    help="--auto: stop after N looks in a row without object or tray")
    ap.add_argument("--interval", type=float, default=2.0, help="--auto: seconds between looks")
    ap.add_argument("--camera", choices=("serial", "usb"), help="override camera.source in the config")
    ap.add_argument("--preview", action="store_true",
                    help="live camera window with detections and markers; no planning or motion")
    ap.add_argument("--aruco-preview", action="store_true",
                    help="live camera window with ArUco only, no YOLO; no motion")
    ap.add_argument("--calibrate-homography", action="store_true",
                    help="fit the table homography from ArUco calibration markers and exit")
    args = ap.parse_args()
    cfg = load_config(args.config)
    cfg["camera"]["source"] = args.camera or cfg["camera"]["source"]

    if args.calibrate_homography:
        calibrate_homography(cfg)
        return
    if args.aruco_preview:
        preview(cfg, None)
        return
    if not args.object:
        ap.error("give the YOLO class name of the object to pick")
    if args.preview:
        preview(cfg, args.object)
        return

    if args.execute:
        missing = [name for name, ok in (("arm.measured", cfg["arm"]["measured"]),
                                         ("table.measured", cfg["table"]["measured"]),
                                         ("joints.calibrated", cfg["joints"]["calibrated"])) if not ok]
        if missing:
            raise SystemExit("Blocked --execute: measure/calibrate first, then set true: " + ", ".join(missing))

    ik = ArmIK(cfg["arm"])
    ddqn = kd.load_model(str(rel(cfg["ddqn"]["checkpoint"])))
    joints = Joints(cfg["joints"], cfg["gripper"])

    dry_run = not args.execute
    if dry_run:
        print("DRY RUN: no servo output. Add --execute to move the arm.")
        pca = DryRunQuiet()
    else:
        what = "keep picking until nothing is left" if args.auto else "do one pick"
        if input(f"Arm resting at home, area clear, power on? It will {what}. "
                 "Type 'yes' to move: ").strip() != "yes":
            print("Cancelled.")
            return
        hw = cfg["hardware"]
        pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)

    cycles = misses = 0
    try:
        while True:
            # In auto mode the arm holds the home pose between picks, so it never
            # sags and the next cycle starts from a known pose.
            last = not args.auto or (args.max_cycles and cycles + 1 >= args.max_cycles)
            try:
                run_cycle(cfg, args, ik, ddqn, joints, pca, dry_run,
                          cut_pwm=cfg["hardware"]["cut_pwm_at_end"] and last)
            except TargetNotFound as e:
                if not args.auto:
                    raise SystemExit(f"{e} Not picking.")
                misses += 1
                print(f"  {e} ({misses}/{args.max_misses})")
                if misses >= args.max_misses:
                    print("Nothing more to pick; stopping.")
                    if cycles and cfg["hardware"]["cut_pwm_at_end"]:
                        pca.all_off()  # the arm is resting at home
                    break
                time.sleep(args.interval)
                continue
            cycles, misses = cycles + 1, 0
            print(f"Pick {cycles} done.")
            if last:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped. Servos are HOLDING the last pose; cut power or press ALL OFF when safe.")
        return
    finally:
        pca.close()
    print(f"Done: {cycles} pick(s).")


class DryRunQuiet(DryRunPca9685):
    """Dry-run output that only reports gripper and power commands, not every pose."""

    def set_pulse_us(self, ch, us):
        pass

    def all_off(self):
        print("[dry-run] all outputs off")


if __name__ == "__main__":
    main()
