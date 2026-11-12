#!/usr/bin/env python3
"""Approach, grip and lift with the camera mounted on top of the gripper.

Sequence:
  drive to the start pose (elbow 70, the rest 90) with the gripper open
  -> sweep down with the shoulder and elbow, again and again, until YOLO sees the selected class
  -> aim until it sits at the target point in the image
  -> hold still until it has been seen commit_detections times there
  -> go down to grip height without looking again
  -> close the gripper, then lift

Usage (run with the .venv-cpu interpreter):
  python visual_pick.py bananapi --show             # dry run: real camera, no servo output
  python visual_pick.py bananapi --detect --show --execute   # move toward the object, no grip
  python visual_pick.py --direction-test            # once: learn which way each servo moves
  python visual_pick.py --standby                   # ramp back to the start pose from wherever it stopped
  python visual_pick.py --rest                      # ramp to the raised rest pose
  python visual_pick.py bananapi --learn-color      # save an object's colour from the camera
  python visual_pick.py relay_module --teach --show --execute   # learn where the jaws' centre is in the image
  python visual_pick.py --grip-test                 # check a hand-placed object stays gripped through moves
  python visual_pick.py relay_module --color --detect --show --execute   # find it by colour, no grip
  python visual_pick.py relay_module --color --show --execute            # find by colour, grip and lift
  python visual_pick.py relay_module --color --auto --show --execute     # repeat pick and drop until none are left
  python visual_pick.py relay_module --auto --show --execute             # the same, YOLO confirmed by colour
  python visual_pick.py bananapi --show --execute   # move the real arm
  python visual_pick.py unknown --detect --show     # guess: whatever stands out from the table

Safety:
- Hobby servos give no position feedback, so the first command jumps to the start
  pose unless the arm is already there (Standby, then Close & hold in
  servo_control_app.py). The joints go one at a time; hold the arm when it is unpowered.
- Travel is limited by the limits section of the config, and the estimated tip
  height never goes below min_tip_height_mm. The way down is blind: the gripper
  closes at grip height whether or not the object is there.
- After a pick run, and on Ctrl+C, q in the camera window, SIGTERM or SIGQUIT,
  the arm ramps to the raised rest pose (rest_servo_deg), opens the gripper and then
  cuts PWM to every servo (limp); an object still held is dropped there. A second Ctrl+C during that move holds the arm
  still instead. SIGKILL and SIGSTOP cannot be caught, so the arm just holds its pose.

Tuning guide. Change numbers in the config, not here, then rerun; no rebuild needed.
Test with --detect first: it does everything up to grip height but does not grip.
A = "approach" and L = "limits" in calibration/visual_pick_config.json;
P = calibration/pick_place_config.json. "Base side" means toward the arm's base.

  What you see                     Change                       Effect
  -------------------------------  ---------------------------  ------------------------------
  Jaws land short, on base side    A gripper_offset_mm  up      1 mm up = jaws 1 mm further out
  Jaws land too far out            A gripper_offset_mm  down    1 mm down = jaws 1 mm closer in
  Jaws land left of the object     A target_u_frac      down    try 0.01 steps; arm turns right
  Jaws land right of the object    A target_u_frac      up      try 0.01 steps; arm turns left
  Jaws hit the table               A grip_tip_mm        up      1 mm up = stops 1 mm higher
  Jaws close above the object      A grip_tip_mm        down    keep it above L min_tip_height_mm
  Object dragged or hits on the    A align_tip_mm       up      height for the move out over
    way out over it                                             the object
  Object lifted too little/much    "grip" lift_mm               mm straight up after gripping
  Object shaken loose when carried "grip" carry_deg_per_s down  slower moves while holding
  Grip too weak                    P gripper grip_deg   down    small steps; below 50 risks a stall
  Jaws do not open enough          P gripper open_deg   up      150 is the known open angle
  Whole arm too slow or too jerky  A ramp_deg_per_s             degrees per second, not holding
  Dropped too high or too low      "place" drop_tip_mm          height of release at the marker
  Marker not found                 "place" scan_base_deg        base angles to look from
                                   "place" scan_rows            extra lower passes, in degrees
  Drops short of or past marker    A gripper_offset_mm          shared with picking, so prefer
                                                                moving the marker
  Aiming hunts back and forth      A aim_gain_deg, aim_turn_frac down, or aim_deadband up
  Aiming stops too early           A aim_deadband       down    0.05 = 5% of the image
  Camera cannot see the table      "hold_servo_deg" wrist_pitch and "start_servo_deg" elbow;
                                   also set STANDBY_DEG in servo_control_app.py to match
  Object not recognised, by colour "color" classes hsv_low, hsv_high, min_area_frac
  Sees objects that are not there  "detection" follow_confidence up (YOLO only)
  unknown picks the table          "guess" table_hsv_low/high: widen it to cover the whole
                                   table (unknown = the object YOLO has no class for)
  unknown misses an object near    "guess" view_frac up, or min_area_frac down; a green
    the edge of the image          object on the green table cannot be seen
  Arm stops at a joint limit       L *_max_deg; shoulder also needs P joints max_deg raised

Heights come from L tip_offset_mm, set from one ruler measurement: 80 mm real when the
model said 44 mm. If every height is off by the same amount, change tip_offset_mm by the
error instead of the values above. Low heights can still differ from the printed tip value.
"""

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from pick_and_place import (  # noqa: E402
    DEFAULT_CONFIG, DryRunQuiet, aruco_detector, load_config, open_camera, rel,
)
from servo_control_app import Pca9685, angle_to_us, us_to_angle  # noqa: E402

VISUAL_CONFIG = ROOT / "calibration" / "visual_pick_config.json"
TICK_S = 0.02
DIRECTION_TEST_DEG = 8.0
START_GAP_S = 0.4
NEAR_LIMIT = 0.25  # how far, in image widths, a tracked blob may move between looks
GUESS_CLASS = "unknown"  # object name for something YOLO has no class for: find_guess finds it
REST_SETTLE_S = 1.0  # let the arm stop at the rest pose before cutting PWM


class StopRequested(Exception):
    """q was pressed in the camera window."""


class Terminated(Exception):
    """SIGTERM or SIGQUIT arrived. SIGKILL and SIGSTOP cannot be caught."""


def _raise_terminated(signum, _frame):
    raise Terminated(signal.Signals(signum).name)


class Arm:
    """Base, shoulder and elbow as offsets from neutral: right, forward and down are positive."""

    def __init__(self, pca, main_cfg, vcfg, dry_run):
        self.pca = pca
        self.dry_run = dry_run
        self.gripper = main_cfg["gripper"]
        self.geometry = main_cfg["arm"]
        self.joints = {j["name"]: j for j in main_cfg["joints"]["list"][:3]}
        by_name = {j["name"]: j for j in main_cfg["joints"]["list"]}
        self.held = [(by_name[name], deg) for name, deg in vcfg["hold_servo_deg"].items()]
        rest = vcfg["rest_servo_deg"]
        self.rest_held = [(by_name[name], rest[name]) for name in vcfg["hold_servo_deg"]]
        self.rest_gripper_deg = rest["gripper"]
        self.neutral = vcfg["neutral_servo_deg"]
        # Wrist pitch above 90 tips the tool down from the forearm line.
        self.tool_pitch = vcfg["hold_servo_deg"]["wrist_pitch"] - 90
        d = vcfg["directions"]
        self.signs = {"base": d["base_right_sign"], "shoulder": d["shoulder_forward_sign"],
                      "elbow": d["elbow_down_sign"]}
        lim = vcfg["limits"]
        self.ranges = {"base": (-lim["base_max_deg"], lim["base_max_deg"]),
                       "shoulder": (-lim["shoulder_back_max_deg"], lim["shoulder_forward_max_deg"]),
                       "elbow": (-lim["elbow_up_max_deg"], lim["elbow_down_max_deg"])}
        self.min_tip = lim["min_tip_height_mm"]
        self.tip_offset = lim["tip_offset_mm"]
        self.free_rate = vcfg["approach"]["ramp_deg_per_s"]
        self.carry_rate = vcfg["grip"]["carry_deg_per_s"]
        self.rate = self.free_rate
        self.start = {name: float(self.signs[name] * (deg - self.neutral[name]))
                      for name, deg in vcfg["start_servo_deg"].items()}
        self.pose = dict(self.start)
        self.rest = {name: float(self.signs[name] * (rest[name] - self.neutral[name]))
                     for name in self.start}

    def tip_height(self, pose):
        """Estimated tip height: upper arm up and forearm level at neutral."""
        g = self.geometry
        lean = math.radians(pose["shoulder"])
        tilt = math.radians(pose["shoulder"] + pose["elbow"])
        return (self.tip_offset + g["shoulder_height_mm"] + g["upper_arm_mm"] * math.cos(lean)
                - g["forearm_mm"] * math.sin(tilt)
                - g["tool_length_mm"] * math.sin(tilt + math.radians(self.tool_pitch)))

    def reach(self, pose):
        """Estimated tip distance out from the base axis."""
        g = self.geometry
        lean = math.radians(pose["shoulder"])
        tilt = math.radians(pose["shoulder"] + pose["elbow"])
        return (g["upper_arm_mm"] * math.sin(lean) + g["forearm_mm"] * math.cos(tilt)
                + g["tool_length_mm"] * math.cos(tilt + math.radians(self.tool_pitch)))

    def allowed(self, deltas, min_tip=None):
        """Clip a move to the travel limits; returns the new pose, or None at the height floor."""
        min_tip = self.min_tip if min_tip is None else min_tip
        pose = dict(self.pose)
        for name, delta in deltas.items():
            lo, hi = self.ranges[name]
            pose[name] = min(hi, max(lo, pose[name] + delta))
        lowering = self.tip_height(pose) < self.tip_height(self.pose)
        if lowering and self.tip_height(pose) < min_tip:
            return None
        return pose

    def servo_deg(self, name, offset):
        deg = self.neutral[name] + self.signs[name] * offset
        j = self.joints[name]
        if not j["min_deg"] <= deg <= j["max_deg"]:
            raise SystemExit(f"ABORT: {name} servo {deg:.1f}° is outside {j['min_deg']}–{j['max_deg']}°.")
        return deg

    def send(self, pose):
        for name, offset in pose.items():
            j = self.joints[name]
            self.pca.set_pulse_us(j["channel"], angle_to_us(j["type"], self.servo_deg(name, offset)))
        self.pose = dict(pose)

    def read_pose(self):
        """Take the pose from the pulses the chip is really sending. False if a joint is unpowered."""
        pose = {}
        for name in self.pose:
            j = self.joints[name]
            us = self.pca.read_pulse_us(j["channel"])
            if us is None:
                return False
            pose[name] = self.signs[name] * (us_to_angle(j["type"], us) - self.neutral[name])
        self.pose = pose
        return True

    def ramp_held(self, targets):
        """Move the wrist servos to their angles at the arm's speed; jump any that are unpowered."""
        for j, deg in targets:
            us = self.pca.read_pulse_us(j["channel"])
            now = deg if us is None else us_to_angle(j["type"], us)
            steps = max(1, math.ceil(abs(deg - now) / (self.rate * TICK_S)))
            for i in range(1, steps + 1):
                self.pca.set_pulse_us(j["channel"], angle_to_us(j["type"], now + (deg - now) * i / steps))
                if not self.dry_run:
                    time.sleep(TICK_S)

    def go_start(self):
        """Ramp to the start pose from where the arm is; jump only if it is unpowered."""
        if self.read_pose():
            self.ramp(self.start)
            self.ramp_held(self.held)
            return
        self.pose = dict(self.start)
        commands = [(self.joints[n], self.servo_deg(n, self.pose[n])) for n in self.pose] + self.held
        for j, deg in commands:
            self.pca.set_pulse_us(j["channel"], angle_to_us(j["type"], deg))
            if not self.dry_run:
                time.sleep(START_GAP_S)

    def go_standby(self):
        if not self.read_pose():
            print("A joint has no signal; the arm will jump to the start pose.")
        self.go_start()

    def go_rest(self):
        """Arm raised skyward (rest_servo_deg), open the gripper, then go limp. Ignores the work limits."""
        self.read_pose()
        self.ramp(self.rest)
        self.ramp_held(self.rest_held)
        self.set_gripper(self.rest_gripper_deg)
        self.rate = self.free_rate
        if not self.dry_run:
            time.sleep(REST_SETTLE_S)
            self.pca.all_off()

    def ramp(self, target):
        start = dict(self.pose)
        span = max(abs(target[k] - start[k]) for k in target)
        steps = max(1, math.ceil(span / (self.rate * TICK_S)))
        for i in range(1, steps + 1):
            self.send({k: start[k] + (target[k] - start[k]) * i / steps for k in target})
            if not self.dry_run:
                time.sleep(TICK_S)

    def set_gripper(self, deg, over_s=0.0):
        """Move the gripper; over_s > 0 closes it gradually so one jaw does not knock the object away."""
        g = self.gripper
        start = getattr(self, "gripper_deg", None)
        steps = max(1, int(over_s / TICK_S)) if over_s and start is not None else 1
        for i in range(1, steps + 1):
            now = deg if steps == 1 else start + (deg - start) * i / steps
            self.pca.set_pulse_us(g["channel"], angle_to_us(g["type"], now))
            if steps > 1 and not self.dry_run:
                time.sleep(TICK_S)
        self.gripper_deg = deg
        if not self.dry_run:
            time.sleep(g["settle_s"])

    def open_gripper(self):
        self.set_gripper(self.gripper["open_deg"])
        self.rate = self.free_rate

    def close_gripper(self):
        self.set_gripper(self.gripper["grip_deg"], self.gripper.get("close_s", 0.0))
        self.rate = self.carry_rate  # jerks can knock a held object loose

    def relax_gripper(self):
        if not self.dry_run:
            self.pca.channel_off(self.gripper["channel"])


class Detector:
    def __init__(self, main_cfg, vcfg, class_name, show, use_color=False, moving=False, cam=None, model=None):
        """cam and model let a long-running caller (the dashboard server) share one camera and YOLO."""
        self.palette = vcfg["color"]
        self.guess = class_name == GUESS_CLASS
        self.guess_cfg = vcfg["guess"]
        self.aim = (vcfg["approach"]["target_u_frac"], vcfg["approach"]["target_v_frac"])
        self.arm = None  # set by find_and_approach, so the guess can tell the arm's own parts
        self.marker_finder = None  # blur-tolerant ArUco detector for the guess, made on first use
        self.self_spots = []  # image spots that stayed put while the arm moved: part of the arm
        self.seen = []  # (pose, u, v) since the guess started following its current spot
        self.use_color = use_color or self.guess  # neither needs YOLO
        # YOLO alone sees relays on a tool case, so a class with a colour must match it too.
        self.verify_color = not self.use_color and class_name in self.palette["classes"]
        self.confirmed = False  # YOLO has seen it on this approach; colour alone tracks it from here
        self.track = None  # (u, v) of the blob being followed, so a bigger blob elsewhere is ignored
        if self.guess:
            print("Guessing: picking whatever stands out from the table (no YOLO).")
        elif use_color:
            if class_name not in self.palette["classes"]:
                raise SystemExit(f"No colour set for '{class_name}'. Known: {', '.join(self.palette['classes'])}")
        else:
            import torch
            from ultralytics import YOLO

            torch.set_num_threads(3)  # measured fastest on the Pi 5; the default here is 1
            self.model = model or YOLO(str(rel(main_cfg["yolo"]["weights"])))
            if class_name not in self.model.names.values():
                raise SystemExit(f"'{class_name}' is not a YOLO class. Known: {', '.join(self.model.names.values())}")
            trained = str((self.model.ckpt or {}).get("date", "unknown"))
            print(f"YOLO model trained {trained}.")
            if moving and not self.verify_color:
                raise SystemExit(f"Blocked --execute: '{class_name}' has no colour to confirm YOLO, and YOLO "
                                 "alone sees objects that are not there. Add --trust-yolo to run anyway.")
        self.cfg = vcfg["detection"]
        self.class_name = class_name
        self.show = show
        self.flip = not vcfg["directions"]["image_upright"]
        self.aruco = aruco_detector(main_cfg["aruco"]["dictionary"])
        self.marker_id = main_cfg["aruco"]["tray_marker_id"]
        self.cam = cam or open_camera(main_cfg["camera"])

    def look_marker(self, status):
        """Like look, for the destination marker."""
        return self.look(status, self.find_marker, f"marker {self.marker_id}")

    def look(self, status, find=None, label=None):
        """Newest frame -> (confidence, u_frac, v_frac, area_frac), or None if not seen."""
        import cv2

        while True:
            try:
                frame = self.cam.read()
                break
            except RuntimeError:
                print("  Waiting for the camera...")
        if self.flip:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        find = find or (self.find_guess if self.guess else self.find_color if self.use_color else self.find_yolo)
        best, best_box = find(frame)
        publish = getattr(self.cam, "publish", None)  # the dashboard's shared camera
        if self.show or publish:
            if best_box:
                cv2.rectangle(frame, best_box[:2], best_box[2:], (0, 255, 0), 2)
                cv2.putText(frame, f"{label or self.class_name} {best[0]:.2f}", (best_box[0], max(12, best_box[1] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, status, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if publish:
            publish(frame)
            if self.cam.stop_requested():
                raise StopRequested
        if self.show:
            cv2.imshow("visual_pick (q to stop)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise StopRequested
        return best

    def find_marker(self, frame):
        """Centre of the destination ArUco marker."""
        h, w = frame.shape[:2]
        corners, ids, _ = self.aruco.detectMarkers(frame)
        for marker, marker_id in zip(corners, [] if ids is None else ids.flatten()):
            if marker_id == self.marker_id:
                pts = marker.reshape(4, 2)
                (x1, y1), (x2, y2) = pts.min(axis=0), pts.max(axis=0)
                area = float((x2 - x1) * (y2 - y1)) / (w * h)
                u, v = float(pts[:, 0].mean()) / w, float(pts[:, 1].mean()) / h
                return (1.0, u, v, area), (int(x1), int(y1), int(x2), int(y2))
        return None, None

    def find_yolo(self, frame):
        if self.verify_color:
            # The colour check takes 2 ms and YOLO about 700 ms, so only ask YOLO when there is a blob.
            from_yolo = self.cfg.get("position_source", "color") == "yolo"
            blob, blob_box = self.find_color(frame, self.track if self.confirmed else None)
            if blob is None or (self.confirmed and not from_yolo):
                self.track = blob[1:3] if blob else self.track
                return blob, blob_box
        h, w = frame.shape[:2]
        result = self.model.predict(frame, imgsz=self.cfg["imgsz"], conf=0.1, device="cpu", verbose=False)[0]
        best, best_box = None, None
        for box in result.boxes:
            if result.names[int(box.cls[0])] != self.class_name:
                continue
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            area = (x2 - x1) * (y2 - y1) / (w * h)
            conf = float(box.conf[0])
            usable = self.verify_color or area <= self.cfg["max_box_area_frac"]
            if usable and (best is None or conf > best[0]):
                best = (conf, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h, area)
                best_box = (int(x1), int(y1), int(x2), int(y2))
        if best and self.verify_color:
            # YOLO says what it is; the colour blob under YOLO's box centre says where, since YOLO
            # boxes are often huge. The largest blob may be something else of the same colour,
            # such as the cardboard under the arm.
            blob, blob_box = self.find_color(frame, best[1:3])
            if blob is None:
                return None, None
            self.confirmed = best[0] >= self.cfg["follow_confidence"]
            self.track = blob[1:3]
            if from_yolo and best[3] <= self.cfg["max_box_area_frac"]:
                # Position = centre of YOLO's box: ((x1 + x2) / 2, (y1 + y2) / 2). A box that
                # covers most of the frame says nothing about where the object is, so then
                # the colour blob gives the position instead.
                return best, best_box
            return (best[0],) + blob[1:], blob_box
        return best, best_box

    def find_color(self, frame, near=None):
        """Blob in the class's HSV range: the largest, or with near=(u, v) the one there."""
        import cv2
        import numpy as np

        h, w = frame.shape[:2]
        c = self.palette["classes"][self.class_name]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), tuple(c["hsv_low"]), tuple(c["hsv_high"]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if c.get("circles"):
            return self.find_circle_pair(frame, c, near)
        min_area = c.get("min_area_frac", self.palette["min_area_frac"])
        rects = [cv2.boundingRect(k) for k in contours]
        return self.choose_rect([r for r in rects if r[2] * r[3] / (w * h) >= min_area], w, h, near)

    @staticmethod
    def choose_rect(rects, w, h, near=None):
        """Boxes (x, y, w, h) -> look result for the largest. With near=(u, v), a box that
        contains that point wins; otherwise the closest one within NEAR_LIMIT of it, else nothing."""
        if not rects:
            return None, None
        if near is None:
            x, y, bw, bh = max(rects, key=lambda r: r[2] * r[3])
        else:
            nx, ny = near[0] * w, near[1] * h
            inside = [r for r in rects if r[0] <= nx <= r[0] + r[2] and r[1] <= ny <= r[1] + r[3]]
            dist = lambda r: math.hypot((r[0] + r[2] / 2 - nx) / w, (r[1] + r[3] / 2 - ny) / h)
            if inside:
                x, y, bw, bh = max(inside, key=lambda r: r[2] * r[3])
            elif min(map(dist, rects)) <= NEAR_LIMIT:
                x, y, bw, bh = min(rects, key=dist)
            else:
                return None, None
        area = bw * bh / (w * h)
        return (1.0, (x + bw / 2) / w, (y + bh / 2) / h, area), (x, y, x + bw, y + bh)

    def find_guess(self, frame, near=None):
        """Best guess at an object YOLO has no class for: the solid thing nearest the aim point
        (the middle of the image) that is not the table colour (guess table_hsv, a green mat).
        Only a circle of view_frac of the image round the aim point counts, so clutter at the
        edges and the floor beyond the table never win, and objects touching each other there
        are split by the circle. Thin shapes (cables, drawn lines) and the destination marker
        (found by ArUco, or by looking like black-and-white print) are skipped. Once found, the
        same spot is followed (self.track) until it is lost. A spot that stays put in the image
        while the arm moves (a gripper jaw or cable in view) is part of the arm, and is ignored
        for the rest of the pick."""
        import cv2
        import numpy as np

        g = self.guess_cfg
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        table = cv2.inRange(hsv, tuple(g["table_hsv_low"]), tuple(g["table_hsv_high"]))
        view = np.zeros_like(table)
        ax, ay = int(self.aim[0] * w), int(self.aim[1] * h)
        cv2.circle(view, (ax, ay), int(g["view_frac"] * min(w, h)), 255, -1)
        mask = cv2.bitwise_and(view, cv2.bitwise_not(table))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        # Round the marker: its white edge and anything stuck to it, such as a note.
        near_marker = np.zeros_like(mask)
        for pts in self.marker_quads(frame):
            centre = pts.mean(axis=0)
            cv2.fillConvexPoly(near_marker, ((pts - centre) * 2.0 + centre).astype(np.int32), 255)
        rects = []
        for k in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            x, y, bw, bh = cv2.boundingRect(k)
            if near_marker[y:y + bh, x:x + bw][mask[y:y + bh, x:x + bw] > 0].any():
                continue
            if (cv2.contourArea(k) / (bw * bh) < g["min_fill"] or min(bw, bh) < g["min_size_px"]
                    or not g["min_area_frac"] <= bw * bh / (w * h) <= g["max_area_frac"]):
                continue
            inside = mask[y:y + bh, x:x + bw] > 0  # this region's own pixels, not the table round it
            s, v = hsv[y:y + bh, x:x + bw, 1][inside], hsv[y:y + bh, x:x + bw, 2][inside]
            # An ArUco marker the detector missed (blurred, or a note stuck on it): white paper with
            # some black print. A white lid is as bright but has no black; measured on the arm's frames.
            print_like = ((s < 60).mean() > 0.75 and (v > 170).mean() > g["paper_bright_frac"]
                          and (v < 80).mean() > g["paper_dark_frac"])
            u, v = (x + bw / 2) / w, (y + bh / 2) / h
            on_arm = any(math.hypot(u - su, v - sv) < g["arm_spot_frac"] for su, sv in self.self_spots)
            if not print_like and not on_arm:
                rects.append((x, y, bw, bh))
        if self.track is None:
            self.seen = []
        near = near or self.track
        if near is None and rects:  # first look: whatever is in front of the camera
            rects = [min(rects, key=lambda r: math.hypot(r[0] + r[2] / 2 - ax, r[1] + r[3] / 2 - ay))]
        best, box = self.choose_rect(rects, w, h, near)
        self.track = best[1:3] if best else self.track
        if best and self.arm is not None:
            self.seen.append((dict(self.arm.pose), best[1], best[2]))
            first, u0, v0 = self.seen[0]
            # Only base turns count: the camera slide (shoulder and elbow in opposite directions)
            # moves the joints a lot but the picture little.
            moved = abs(self.arm.pose["base"] - first["base"])
            if moved >= g["arm_check_deg"] and math.hypot(best[1] - u0, best[2] - v0) < g["arm_spot_frac"]:
                print(f"  Ignoring u{best[1]:.2f} v{best[2]:.2f}: it stayed put while the arm moved "
                      f"{moved:.0f} deg, so it is part of the arm.")
                self.self_spots.append(best[1:3])
                self.track, self.seen = None, []
                return None, None
        return best, box

    def marker_quads(self, frame):
        """Corners of the destination marker, found more often than with find_marker's detector:
        it accepts blur and more bit errors, also tries a half-size image, and keeps only the
        marker id, since the looser settings also report ids that are not there."""
        import cv2

        if self.marker_finder is None:
            p = cv2.aruco.DetectorParameters()
            p.adaptiveThreshWinSizeMin, p.adaptiveThreshWinSizeMax, p.adaptiveThreshWinSizeStep = 3, 53, 6
            p.errorCorrectionRate = 1.0
            p.polygonalApproxAccuracyRate = 0.08
            p.minMarkerPerimeterRate = 0.02
            self.marker_finder = cv2.aruco.ArucoDetector(self.aruco.getDictionary(), p)
        quads = []
        for scale in (1.0, 0.5):
            img = frame if scale == 1.0 else cv2.resize(frame, None, fx=scale, fy=scale)
            corners, ids, _ = self.marker_finder.detectMarkers(img)
            quads += [c.reshape(4, 2) / scale for c, i in zip(corners, [] if ids is None else ids.flatten())
                      if i == self.marker_id]
            if quads:
                break
        return quads

    def find_circle_pair(self, frame, c, near=None):
        """Two round parts of about the same size side by side on a board of the class colour,
        e.g. the HC-SR04 transducers. The board colour alone is unreliable (it washes out under
        bright light), so the circles find it and the colour only confirms it."""
        import cv2
        import numpy as np

        h, w = frame.shape[:2]
        gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
        found = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 40, param1=120, param2=40,
                                 minRadius=12, maxRadius=90)
        if found is None:
            return None, None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        board = cv2.inRange(hsv, tuple(c["board_hsv_low"]), tuple(c["board_hsv_high"]))
        best = None
        circles = [tuple(map(float, k)) for k in found[0][:8]]
        for i, (x1, y1, r1) in enumerate(circles):
            for x2, y2, r2 in circles[i + 1:]:
                gap = math.hypot(x2 - x1, y2 - y1) / ((r1 + r2) / 2)
                if max(r1, r2) / min(r1, r2) > 1.4 or not 1.8 <= gap <= 4.5:
                    continue
                r = (r1 + r2) / 2
                x0, y0 = int(max(0, min(x1, x2) - r)), int(max(0, min(y1, y2) - r))
                xe, ye = int(min(w, max(x1, x2) + r)), int(min(h, max(y1, y2) + r))
                cover = board[y0:ye, x0:xe].mean() / 255 if xe > x0 and ye > y0 else 0
                if cover < c["board_min_cover"]:
                    continue
                u, v = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
                # Tracking: prefer the pair nearest the last position; otherwise the most board.
                score = -math.hypot(u - near[0], v - near[1]) if near else cover
                if best is None or score > best[0]:
                    best = (score, (1.0, u, v, (xe - x0) * (ye - y0) / (w * h)), (x0, y0, xe, ye))
        if best is None or (near and -best[0] > NEAR_LIMIT):
            return None, None
        return best[1], best[2]

    def close(self):
        self.cam.close()
        if self.show:
            import cv2

            cv2.destroyAllWindows()


def search_step(arm, vcfg):
    """One step down the sweep; from the bottom, back to the start pose. True on that restart."""
    a = vcfg["approach"]
    deltas = {"elbow": a["search_elbow_step_deg"], "shoulder": a["search_shoulder_step_deg"]}
    target = arm.allowed(deltas, vcfg["limits"]["search_min_tip_mm"])
    restarted = target is None or target == arm.pose
    if restarted:
        print("  Bottom of the sweep; starting again from the top.")
        target = arm.start
    arm.ramp(target)
    if not arm.dry_run:
        time.sleep(a["search_settle_s"])
    return restarted


def aim_step(arm, det, vcfg, min_tip=None):
    """One correction toward the target point. False when aimed, None if the base cannot turn far enough.

    The base turn and the slide out or back happen in the same move, so each look corrects
    both directions at once.
    """
    a = vcfg["approach"]
    u_err = det[1] - a["target_u_frac"]
    v_err = det[2] - a["target_v_frac"]
    deltas = {}
    if abs(u_err) > a["aim_deadband"]:
        deltas["base"] = a["aim_turn_frac"] * a["base_deg_per_u"] * u_err
    if abs(v_err) > a["aim_deadband"] and (a.get("aim_both_axes", True) or not deltas):
        # aim_both_axes false: turn the base first, slide only once it is lined up.
        # High in the image means it is further out. Shoulder and elbow turn by equal and
        # opposite amounts so the camera slides out or back without changing its tilt;
        # tilting instead would push the object off the top of the image.
        step = a["aim_gain_deg"] * v_err
        deltas.update(shoulder=-step, elbow=step)
    if not deltas:
        return False
    target = arm.allowed(deltas, min_tip)
    if (target is None or target == arm.pose) and "base" in deltas and len(deltas) > 1:
        target = arm.allowed({"base": deltas["base"]}, min_tip)  # cannot slide any further: just turn
    if target is None or target == arm.pose:
        return None if "base" in deltas else False  # too low to get closer: lock on from here
    arm.ramp(target)
    if not arm.dry_run:
        time.sleep(a["aim_settle_s"])
    return True


def move_tip(arm, height_mm, reach_mm, step_mm=5.0):
    """Move the tip in a straight line to this height and reach, in small steps."""
    for _ in range(60):
        pose = arm.pose
        dz = height_mm - arm.tip_height(pose)
        dr = reach_mm - arm.reach(pose)
        if abs(dz) < 1.0 and abs(dr) < 1.0:
            return
        scale = min(1.0, step_mm / math.hypot(dz, dr))
        dz, dr = dz * scale, dr * scale
        # Numeric Jacobian of (height, reach) against (shoulder, elbow), per degree.
        moved = [{**pose, name: pose[name] + 1.0} for name in ("shoulder", "elbow")]
        zs, ze = (arm.tip_height(m) - arm.tip_height(pose) for m in moved)
        rs, re = (arm.reach(m) - arm.reach(pose) for m in moved)
        det = zs * re - ze * rs
        if abs(det) < 1e-6:
            break
        target = arm.allowed({"shoulder": (dz * re - ze * dr) / det, "elbow": (zs * dr - dz * rs) / det})
        if target is None or target == pose:
            break
        arm.ramp(target)
    print(f"  Stopped short: tip ~{arm.tip_height(arm.pose):.0f} mm high, "
          f"{reach_mm - arm.reach(arm.pose):.0f} mm short of the reach.")


def descend(arm, u, vcfg, v=None):
    """Turn to the object once, then go down to grip height without looking again.

    u corrects the base turn; v, if given, corrects how far out the jaws go, so the small
    error left after aiming is taken out here instead of with more looks.
    """
    a = vcfg["approach"]
    # base_offset_deg corrects a steady sideways miss of the jaws (positive turns right).
    arm.ramp(arm.allowed({"base": a["base_deg_per_u"] * (u - a["target_u_frac"]) + a.get("base_offset_deg", 0.0)}))
    # The object is under the lens and the jaws sit behind it: go out over it above
    # its height, then straight down.
    reach = arm.reach(arm.pose) + a["gripper_offset_mm"]
    if v is not None:
        reach += (a["target_v_frac"] - v) * a.get("mm_per_image_height", 0.0)  # high in the image = further out
    move_tip(arm, a["align_tip_mm"], reach)
    rate, arm.rate = arm.rate, a.get("final_descent_deg_per_s", arm.rate)  # slow near the object
    try:
        move_tip(arm, a["grip_tip_mm"], reach)
    finally:
        arm.rate = rate


def approach(arm, look, vcfg, clock=time.monotonic, max_sweeps=None):
    """Search, aim at the object, and after commit_detections sightings go down.

    Returns (found, reason).

    max_sweeps ends the search with "nothing found"; None keeps searching for good.
    """
    a, d = vcfg["approach"], vcfg["detection"]
    sightings = []
    last_seen = None
    sweeps = aim_steps = 0
    while True:
        status = "aiming" if last_seen is not None else "searching"
        det = look(status)
        now = clock()
        box = f"conf {det[0]:.2f} box {det[3] * 100:3.0f}% u{det[1]:.2f} v{det[2]:.2f}" if det else "no detection"
        print(f"  {status:9s} {box:34s} tip ~{arm.tip_height(arm.pose):3.0f} mm "
              f"pose b{arm.pose['base']:+.0f} s{arm.pose['shoulder']:+.0f} e{arm.pose['elbow']:+.0f}")
        if det is not None and det[0] >= d["follow_confidence"]:
            last_seen = now
            moved = aim_step(arm, det, vcfg, a.get("aim_min_tip_mm")) if a["aim_before_pick"] else False
            if moved is None:
                return False, "object is outside the base travel"
            if moved:
                sightings = []
                aim_steps += 1
                if aim_steps > a["max_aim_steps"]:
                    return False, "could not aim at the object"
                continue
            sightings.append(det)
            if len(sightings) >= a["commit_detections"]:
                u = sum(s[1] for s in sightings) / len(sightings)
                v = sum(s[2] for s in sightings) / len(sightings)
                print(f"Detected {len(sightings)} times at u{u:.2f} v{v:.2f}. Going down.")
                arm.aim_pose = dict(arm.pose)  # --teach comes back here to look again
                descend(arm, u, vcfg, v)
                return True, f"at grip height, tip ~{arm.tip_height(arm.pose):.0f} mm"
        elif last_seen is None:
            sweeps += search_step(arm, vcfg)
            if max_sweeps and sweeps >= max_sweeps:
                return False, "nothing found"
        elif now - last_seen > a["lost_timeout_s"]:
            # Hold still until then; it may just be a bad frame.
            return False, "object lost"


def find_and_approach(arm, detector, vcfg, max_sweeps=None):
    """Run approach, going back to the start pose to look again whenever the object is lost."""
    detector.arm = arm
    detector.confirmed, detector.track = False, None
    ok, reason = approach(arm, detector.look, vcfg, max_sweeps=max_sweeps)
    while reason == "object lost":
        print("Object lost. Going back to the start pose to look again.")
        arm.ramp(arm.start)
        detector.confirmed, detector.track = False, None
        ok, reason = approach(arm, detector.look, vcfg, max_sweeps=max_sweeps)
    return ok, reason


def grip_and_lift(arm, reason, vcfg):
    g = vcfg["grip"]
    print(f"Stopping: {reason}.")
    if not arm.dry_run:
        time.sleep(g["stop_settle_s"])
    print("Closing gripper.")
    arm.close_gripper()
    print("Lifting.")
    # Straight up, so the object is not dragged along the table.
    move_tip(arm, arm.tip_height(arm.pose) + g["lift_mm"], arm.reach(arm.pose))


def run_test(arm, detector, vcfg):
    print("Opening gripper at the start pose.")
    arm.go_start()
    arm.open_gripper()
    ok, reason = find_and_approach(arm, detector, vcfg)
    if not ok:
        print(f"Not gripping: {reason}. Returning to the start pose with the gripper open.")
        arm.ramp(arm.start)
        arm.relax_gripper()
        return False

    grip_and_lift(arm, reason, vcfg)
    print("Lifted. Check whether the object is in the gripper.")
    if not arm.dry_run:
        input("Hold a hand under it, then press Enter to open the gripper and return to the start pose: ")
    arm.open_gripper()
    arm.relax_gripper()
    arm.ramp(arm.start)
    return True


def scan_for_marker(arm, detector, vcfg):
    """Turn across the table in rows until the marker is seen; False if it never is."""
    p = vcfg["place"]
    for row in p["scan_rows"]:
        for base in p["scan_base_deg"]:
            arm.ramp({**arm.start, "base": base, "shoulder": arm.start["shoulder"] + row,
                      "elbow": arm.start["elbow"] + row})
            if not arm.dry_run:
                time.sleep(vcfg["approach"]["search_settle_s"])
            for _ in range(p["looks_per_stop"]):
                det = detector.look_marker("looking for marker")
                if det:
                    return True
    return False


def place_on_marker(arm, detector, vcfg, memory):
    """Carry the held object to the ArUco marker and release it. False if it stays in the gripper."""
    p, a = vcfg["place"], vcfg["approach"]
    print(f"YOLO off. Looking for ArUco id {detector.marker_id} with OpenCV.")
    if scan_for_marker(arm, detector, vcfg):
        for _ in range(a["max_aim_steps"]):
            det = detector.look_marker("aiming at marker")
            if det is None:
                continue
            print(f"  marker box {det[3] * 100:3.0f}% u{det[1]:.2f} v{det[2]:.2f} "
                  f"tip ~{arm.tip_height(arm.pose):3.0f} mm pose b{arm.pose['base']:+.0f}")
            moved = aim_step(arm, det, vcfg, p["carry_tip_mm"])
            if moved is None:
                print("Marker is outside the base travel.")
                return False
            if not moved:
                memory.update(base=arm.pose["base"], reach=arm.reach(arm.pose) + a["gripper_offset_mm"])
                break
        else:
            print("Could not finish aiming; placing from the best aim.")
            memory.update(base=arm.pose["base"], reach=arm.reach(arm.pose) + a["gripper_offset_mm"])
    elif memory:
        print("Marker not seen; using the place it was last found.")
        arm.ramp({**arm.start, "base": memory["base"]})
    else:
        print("Marker not found.")
        return False
    move_tip(arm, p["drop_tip_mm"], memory["reach"])
    print("Releasing.")
    arm.open_gripper()
    arm.relax_gripper()
    print("Placed. YOLO back on.")
    return True


def auto_run(arm, detector, vcfg, max_picks, wait_for_hand=True):
    """Pick, carry to the marker and drop, until nothing is left or max_picks is reached."""
    auto = vcfg["auto"]
    print("Automatic mode: pick, carry to the marker, drop, until nothing is left. Ctrl+C to stop.")
    arm.go_start()
    picks = 0
    memory = {}
    while not max_picks or picks < max_picks:
        arm.open_gripper()
        ok, reason = find_and_approach(arm, detector, vcfg, max_sweeps=auto["max_sweeps"])
        if not ok:
            arm.ramp(arm.start)
            arm.relax_gripper()
            print(f"Stopping: {reason}.")
            break
        grip_and_lift(arm, reason, vcfg)
        if not place_on_marker(arm, detector, vcfg, memory):
            print("Stopping with the object still in the gripper.")
            if not arm.dry_run and wait_for_hand:
                input("Hold a hand under it, then press Enter to open the gripper: ")
            arm.open_gripper()
            arm.relax_gripper()
            arm.ramp(arm.start)
            break
        arm.ramp(arm.start)
        if not arm.dry_run:
            time.sleep(vcfg["approach"]["search_settle_s"])
        picks += 1
        print(f"Pick {picks} done.")
    print(f"Finished: {picks} pick(s).")
    return picks


def grip_test(arm, vcfg):
    """Hold a hand-placed object through each kind of move, to see which one loosens it."""
    arm.go_start()
    arm.open_gripper()
    input("Put the object between the jaws, hold it there, then press Enter: ")
    arm.close_gripper()
    stages = (("holding still", dict(arm.start)),
              ("turning the base", {**arm.start, "base": 30.0}),
              ("back to centre", dict(arm.start)),
              ("reaching out", {**arm.start, "shoulder": 40.0, "elbow": -10.0}),
              ("back to the start pose", dict(arm.start)))
    for name, pose in stages:
        input(f"Next: {name}. Press Enter, keep a hand under the object: ")
        arm.ramp(pose)
        time.sleep(2.0)
        print("  Did the object stay gripped?")
    input("Press Enter to open the gripper: ")
    arm.open_gripper()
    arm.relax_gripper()


def teach(arm, detector, vcfg, class_name, config_path):
    """Learn where an object sits in the image when it is exactly between the jaws.

    The arm aims and goes down as for a pick, without gripping. The user slides the object
    to the centre of the open jaws; the arm goes back up to where it aimed and looks again.
    That image position becomes this object's target, so later picks aim at it directly and
    no camera-to-jaw distance has to be guessed.
    """
    print("Teach: the arm goes down beside the object without gripping.")
    arm.go_start()
    arm.open_gripper()
    ok, reason = find_and_approach(arm, detector, vcfg)
    if not ok:
        print(f"Could not reach the object: {reason}.")
        return
    input("Slide the object so it sits centred between the open jaws, hands clear, then press Enter: ")
    a = vcfg["approach"]
    move_tip(arm, a["align_tip_mm"], arm.reach(arm.pose))  # straight up, clear of the object
    arm.ramp(arm.aim_pose)
    if not arm.dry_run:
        time.sleep(1.0)
    detector.confirmed, detector.track = False, None
    seen = [d for d in (detector.look("teaching") for _ in range(5)) if d]
    if len(seen) < 3:
        print("The object is not clearly visible from the aiming pose; nothing saved.")
        return
    u = sorted(d[1] for d in seen)[len(seen) // 2]
    v = sorted(d[2] for d in seen)[len(seen) // 2]
    print(f"Centred between the jaws, {class_name} appears at u{u:.3f} v{v:.3f} "
          f"(was aiming for u{a['target_u_frac']:.3f} v{a['target_v_frac']:.3f}).")
    if not ask(f"Save this as the aim point for {class_name}?"):
        return
    raw = load_config(config_path)
    entry = raw.setdefault("objects", {}).setdefault(class_name, {}).setdefault("approach", {})
    entry.update(target_u_frac=round(u, 4), target_v_frac=round(v, 4), base_offset_deg=0.0)
    Path(config_path).write_text(json.dumps(raw, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Saved to {config_path} (objects.{class_name}.approach).")


def detect_only(arm, detector, vcfg):
    """Find the object and go down to it, stopping there without gripping."""
    print("Going to the start pose, then finding the object and going down to it. Ctrl+C to stop.")
    arm.go_start()
    ok, reason = find_and_approach(arm, detector, vcfg)
    if ok:
        print(f"Detected: {reason}. Servos are holding this pose.")
    else:
        print(f"Stopped: {reason}. Returning to the start pose.")
        arm.ramp(arm.start)


def learn_color(frames, margin=(8, 40, 50), centre_frac=0.3):
    """HSV range of whatever fills the centre of the frames, widened by margin.

    Returns (hsv_low, hsv_high, hue_spread). A hue spread over about 40 means the centre
    holds several colours (or red, which wraps around 0), so the range will catch too much.
    """
    import cv2
    import numpy as np

    h, w = frames[0].shape[:2]
    y0, x0 = int(h * (1 - centre_frac) / 2), int(w * (1 - centre_frac) / 2)
    px = np.concatenate([cv2.cvtColor(f, cv2.COLOR_BGR2HSV)[y0:h - y0, x0:w - x0].reshape(-1, 3)
                         for f in frames])
    lo, hi = np.percentile(px, 10, axis=0), np.percentile(px, 90, axis=0)
    top = (179, 255, 255)
    low = [int(max(0, lo[i] - margin[i])) for i in range(3)]
    high = [int(min(top[i], hi[i] + margin[i])) for i in range(3)]
    return low, high, float(hi[0] - lo[0])


def learn_color_command(main_cfg, vcfg, class_name, config_path):
    """Sample the object held in the middle of the camera view and save its colour range."""
    input(f"Put the {class_name} flat on the table, filling the middle of the camera view, "
          "then press Enter: ")
    cam = open_camera(main_cfg["camera"])
    try:
        frames = [cam.read() for _ in range(6)][-4:]
    finally:
        cam.close()
    low, high, spread = learn_color(frames)
    print(f"HSV low {low}, high {high} (hue spread {spread:.0f})")
    if spread > 40:
        print("WARNING: the middle of the image holds several colours, so this range is wide "
              "and may also match the table or other objects.")
    vcfg["color"]["classes"][class_name] = {"hsv_low": low, "hsv_high": high}
    d = Detector.__new__(Detector)
    d.palette, d.class_name = vcfg["color"], class_name
    found = d.find_color(frames[-1])[0]
    print("Check on the same view:", f"found at u{found[1]:.2f} v{found[2]:.2f}, "
          f"{found[3] * 100:.0f}% of the frame" if found else "nothing found; adjust and retry")
    if found and ask(f"Save this colour for {class_name}?"):
        Path(config_path).write_text(json.dumps(vcfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"Saved to {config_path}. YOLO picks of {class_name} are now confirmed by this colour.")


def apply_object_settings(vcfg, class_name):
    """Overlay the "objects" entry for this class on the shared sections, e.g. heights for a tall box."""
    for section, values in vcfg.get("objects", {}).get(class_name, {}).items():
        if isinstance(values, dict):
            vcfg[section].update({k: v for k, v in values.items() if not k.startswith("_")})


def ask(question):
    return input(f"{question} [y/n]: ").strip().lower().startswith("y")


def direction_test(main_cfg, vcfg, config_path):
    """Nudge each joint from the start pose and record which way it went."""
    print(f"Each joint moves {DIRECTION_TEST_DEG:.0f}° from the start pose and back. Hold the arm, area clear.")
    if not ask("Servo power on, area clear, hand on the arm (it may jump to the start pose)?"):
        print("Cancelled.")
        return
    hw = main_cfg["hardware"]
    pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)
    d = vcfg["directions"]
    d.update(base_right_sign=1, shoulder_forward_sign=1, elbow_down_sign=1)
    arm = Arm(pca, main_cfg, vcfg, dry_run=False)
    home = dict(arm.pose)
    questions = (("shoulder", "shoulder_forward_sign", "Did the upper arm lean FORWARD?"),
                 ("elbow", "elbow_down_sign", "Did the forearm move DOWN?"),
                 ("base", "base_right_sign", "Did the arm turn RIGHT (seen from behind the arm)?"))
    try:
        arm.go_start()
        for name, key, question in questions:
            arm.ramp({**home, name: home[name] + DIRECTION_TEST_DEG})
            d[key] = 1 if ask(question) else -1
            arm.ramp(home)
        d["image_upright"] = ask("In --preview, is the image the right way up?")
    finally:
        pca.close()
    d["confirmed"] = True
    Path(config_path).write_text(json.dumps(vcfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Saved to {config_path}: {d}")


def main():
    ap = argparse.ArgumentParser(description="Eye-in-hand YOLO approach, grip and lift")
    ap.add_argument("object", nargs="?", help="YOLO class name to pick, e.g. bananapi")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--visual-config", default=str(VISUAL_CONFIG))
    ap.add_argument("--execute", action="store_true", help="drive the real servos (default: dry run)")
    ap.add_argument("--show", action="store_true", help="show the annotated camera window")
    ap.add_argument("--detect", action="store_true",
                    help="find the object and go down to it, but do not grip")
    ap.add_argument("--auto", action="store_true", help="repeat pick and drop until nothing is left")
    ap.add_argument("--max-picks", type=int, default=0, help="--auto: stop after N picks (0 = no limit)")
    ap.add_argument("--grip-test", action="store_true",
                    help="hold a hand-placed object through base and reach moves, no camera")
    ap.add_argument("--standby", action="store_true", help="ramp back to the start pose and exit")
    ap.add_argument("--rest", action="store_true", help="ramp to the raised rest pose and exit")
    ap.add_argument("--camera", choices=("serial", "usb"), help="override camera.source in the config")
    ap.add_argument("--trust-yolo", action="store_true", help="allow --execute on a class with no colour to confirm it")
    ap.add_argument("--teach", action="store_true",
                    help="learn the object's aim point by centring it between the jaws by hand")
    ap.add_argument("--learn-color", action="store_true",
                    help="sample the object's colour from the camera and save it; no motion")
    ap.add_argument("--color", action="store_true", help="find the object by colour instead of YOLO")
    ap.add_argument("--direction-test", action="store_true",
                    help="nudge each joint to learn its direction, save the result and exit")
    args = ap.parse_args()
    main_cfg = load_config(args.config)
    vcfg = load_config(args.visual_config)
    main_cfg["camera"]["source"] = args.camera or main_cfg["camera"]["source"]

    if args.direction_test:
        direction_test(main_cfg, vcfg, args.visual_config)
        return
    if args.grip_test:
        if ask("Area clear and hand ready to catch the object?"):
            hw = main_cfg["hardware"]
            pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)
            try:
                grip_test(Arm(pca, main_cfg, vcfg, dry_run=False), vcfg)
            except KeyboardInterrupt:
                print("\nStopped. Servos are HOLDING the last pose; hold the arm.")
            finally:
                pca.close()
        return
    if args.rest:
        if ask("Area clear, hand on the arm, and no other program moving it?"):
            hw = main_cfg["hardware"]
            pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)
            try:
                Arm(pca, main_cfg, vcfg, dry_run=False).go_rest()
                print("At rest. Servos are limp.")
            finally:
                pca.close()
        return
    if args.standby:
        if ask("Area clear, hand on the arm, and no other program moving it?"):
            hw = main_cfg["hardware"]
            pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)
            try:
                Arm(pca, main_cfg, vcfg, dry_run=False).go_standby()
                print("At the start pose. Servos are holding it.")
            finally:
                pca.close()
        return
    if not args.object:
        ap.error("give the YOLO class name of the object to pick")
    if args.learn_color:
        learn_color_command(main_cfg, vcfg, args.object, args.visual_config)
        return
    apply_object_settings(vcfg, args.object)

    if not args.execute:
        print("DRY RUN: real camera, no servo output. Add --execute to move the arm.")
        pca = DryRunQuiet()
    else:
        if not vcfg["directions"]["confirmed"]:
            raise SystemExit("Blocked --execute: run python visual_pick.py --direction-test first.")
        if not ask("Servo power on, area clear, hand on the arm (it may jump to the start pose)?"):
            print("Cancelled.")
            return
        hw = main_cfg["hardware"]
        pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)

    # SIGHUP was tried too, but a library in the process swallows it on this Pi.
    for sig in (signal.SIGTERM, signal.SIGQUIT):
        signal.signal(sig, _raise_terminated)
    detector = arm = None
    try:
        detector = Detector(main_cfg, vcfg, args.object, args.show, args.color, args.execute and not args.trust_yolo)
        arm = Arm(pca, main_cfg, vcfg, dry_run=not args.execute)
        if args.auto:
            auto_run(arm, detector, vcfg, args.max_picks)
        else:
            if args.teach:
                teach(arm, detector, vcfg, args.object, args.visual_config)
            else:
                (detect_only if args.detect else run_test)(arm, detector, vcfg)
        if not args.detect:
            print("Going to the rest pose.")
            arm.go_rest()
        print("Done.")
    except (KeyboardInterrupt, StopRequested, Terminated) as e:
        print(f"\nStopped ({type(e).__name__} {e}).".replace(" )", ")"))
        if arm is not None:
            print("Going to the rest pose. Press Ctrl+C again to hold still instead.")
            try:
                arm.go_rest()
                print("At rest. Servos are limp.")
            except (KeyboardInterrupt, Terminated):
                print("\nHolding the last pose; hold the arm, then cut power or press ALL OFF.")
    finally:
        if detector:
            detector.close()
        pca.close()


if __name__ == "__main__":
    main()
