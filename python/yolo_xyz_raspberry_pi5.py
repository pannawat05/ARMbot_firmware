#!/usr/bin/env python3
"""YOLO XYZ detector for Raspberry Pi 5; outputs JSON for a DDQN controller.

Calibration requirements:
- H_CAMERA_TO_ROBOT is a 3x3 homography mapping [u_px, v_px] to
  [robot_base_X_mm, robot_base_Y_mm], calibrated at 640x480.
- TABLE_Z_MM is the table surface height relative to robot-base origin.
- GRASP_Z_OFFSET_MM is the gripper reference height above the table per class.
This script detects and reports coordinates only. It does not move servos.
"""
import argparse
import json
import time
from pathlib import Path
from typing import Optional, Any

import cv2
import numpy as np
from ultralytics import YOLO

WEIGHTS_PATH = "best.pt"
HOMOGRAPHY_PATH = "H_camera_to_robot.npy"
IMGSZ = 640
CONFIDENCE = 0.35
IOU_THRESHOLD = 0.45
PIXEL_POINT_MODE = "center"  # "center" or "bottom_center"

# Fill these with measured values; None intentionally means unknown.
TABLE_Z_MM: Optional[float] = None
GRASP_Z_OFFSET_MM = {
    "bananapi": None,
    "relay_module": None,
    "ultrasonic_hc-sr04": None,
}


def load_homography(path: str) -> np.ndarray:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"Missing {p}. Create this from real calibration points first."
        )
    H = np.load(str(p))
    if H.shape != (3, 3) or not np.isfinite(H).all():
        raise ValueError("Homography must be a finite 3x3 matrix.")
    if abs(np.linalg.det(H)) < 1e-12:
        raise ValueError("Homography matrix is singular.")
    return H.astype(np.float64)


def pixel_to_robot_xy(u: float, v: float, H: np.ndarray) -> tuple[float, float]:
    p = np.array([[[u, v]]], dtype=np.float64)
    xy = cv2.perspectiveTransform(p, H)[0, 0]
    return float(xy[0]), float(xy[1])


class YoloXYZDetector:
    """Reusable class for use inside a DDQN process."""

    def __init__(
        self,
        weights_path: str = WEIGHTS_PATH,
        homography_path: str = HOMOGRAPHY_PATH,
        source: Any = 0,
        imgsz: int = IMGSZ,
        confidence: float = CONFIDENCE,
        table_z_mm: Optional[float] = TABLE_Z_MM,
        grasp_z_offsets: Optional[dict] = None,
        pixel_point_mode: str = PIXEL_POINT_MODE,
    ):
        self.model = YOLO(weights_path)
        self.H = load_homography(homography_path)
        self.source = source
        self.imgsz = int(imgsz)
        self.confidence = float(confidence)
        self.table_z_mm = table_z_mm
        self.grasp_z_offsets = (
            GRASP_Z_OFFSET_MM.copy()
            if grasp_z_offsets is None else grasp_z_offsets.copy()
        )
        if pixel_point_mode not in ("center", "bottom_center"):
            raise ValueError("pixel_point_mode must be center or bottom_center")
        self.pixel_point_mode = pixel_point_mode

    def detect_frame(self, frame: np.ndarray) -> dict:
        if frame is None or frame.size == 0:
            raise ValueError("Empty camera frame.")

        # Homography must match 640x480 calibration. Resize incoming frames
        # to this resolution before using H.
        if frame.shape[1] != 640 or frame.shape[0] != 480:
            frame = cv2.resize(frame, (640, 480))

        result = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.confidence,
            iou=IOU_THRESHOLD,
            device="cpu",
            verbose=False,
        )[0]

        detections = []
        for box in result.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].cpu().tolist()]
            class_id = int(box.cls[0].item())
            class_name = result.names[class_id]
            conf = float(box.conf[0].item())

            if self.pixel_point_mode == "bottom_center":
                u, v = (x1 + x2) / 2.0, y2
            else:
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            rx, ry = pixel_to_robot_xy(u, v, self.H)
            z_table = float(self.table_z_mm) if self.table_z_mm is not None else None
            offset = self.grasp_z_offsets.get(class_name)
            z_grasp = (
                z_table + float(offset)
                if z_table is not None and offset is not None else None
            )

            detections.append({
                "class_id": class_id,
                "class_name": class_name,
                "confidence": conf,
                "bbox_xyxy_px": [x1, y1, x2, y2],
                "pixel_point_uv": [float(u), float(v)],
                "robot_X_mm": rx,
                "robot_Y_mm": ry,
                "table_Z_mm": z_table,
                "grasp_Z_mm": z_grasp,
                "xyz_mm": [rx, ry, z_grasp],
                "coordinate_frame": "robot_base",
            })

        target = max(detections, key=lambda d: d["confidence"]) if detections else None
        return {
            "timestamp_unix": time.time(),
            "image_width": 640,
            "image_height": 480,
            "detections": detections,
            "target": target,
            "target_available": target is not None,
            "xyz_complete": bool(
                target is not None and all(v is not None for v in target["xyz_mm"])
            ),
        }

    def open_capture(self):
        src = self.source
        if isinstance(src, str) and src.isdigit():
            src = int(src)
        cap = cv2.VideoCapture(src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            raise RuntimeError(
                f"Cannot open source {self.source}. For ESP32-CAM, pass its actual "
                "stream URL, e.g. http://CAMERA_IP:81/stream if your firmware uses it."
            )
        return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=WEIGHTS_PATH)
    ap.add_argument("--homography", default=HOMOGRAPHY_PATH)
    ap.add_argument("--source", default="0", help="camera index or ESP32-CAM stream URL")
    ap.add_argument("--json-out", default="/tmp/yolo_xyz_state.json")
    ap.add_argument("--table-z-mm", type=float, default=TABLE_Z_MM)
    ap.add_argument("--grasp-z-bananapi", type=float, default=GRASP_Z_OFFSET_MM["bananapi"])
    ap.add_argument("--grasp-z-relay", type=float, default=GRASP_Z_OFFSET_MM["relay_module"])
    ap.add_argument("--grasp-z-ultrasonic", type=float, default=GRASP_Z_OFFSET_MM["ultrasonic_hc-sr04"])
    ap.add_argument("--conf", type=float, default=CONFIDENCE)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    offsets = {
        "bananapi": args.grasp_z_bananapi,
        "relay_module": args.grasp_z_relay,
        "ultrasonic_hc-sr04": args.grasp_z_ultrasonic,
    }
    detector = YoloXYZDetector(
        weights_path=args.weights,
        homography_path=args.homography,
        source=args.source,
        confidence=args.conf,
        table_z_mm=args.table_z_mm,
        grasp_z_offsets=offsets,
    )
    cap = detector.open_capture()
    out = Path(args.json_out)
    out.parent.mkdir(parents=True, exist_ok=True)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("WARN: failed to read frame")
                time.sleep(0.2)
                continue

            payload = detector.detect_frame(frame)
            tmp = out.with_suffix(out.suffix + ".tmp")
            tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(out)  # atomic write for the DDQN reader

            target = payload["target"]
            if target:
                print(
                    f'{target["class_name"]} conf={target["confidence"]:.3f} '
                    f'XY=({target["robot_X_mm"]:.1f}, {target["robot_Y_mm"]:.1f}) mm '
                    f'grasp_Z={target["grasp_Z_mm"]}'
                )
            else:
                print("No target detected")

            if args.show:
                cv2.imshow("YOLO XYZ - press q to quit", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
