#!/usr/bin/env python3
"""Receive raw back-to-back JPEG frames from ESP32-CAM over UART and run YOLO.

The ESP32 firmware just does Serial.write(fb->buf, fb->len) in a loop, so frames
are found by their JPEG markers: SOI (FF D8) ... EOI (FF D9). No length header.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"
MAX_FRAME_BYTES = 300_000
MAX_BUFFER = 1_000_000


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (ไม่พบพอร์ต serial เลย - เช็คสาย USB / ไดรเวอร์ CH340)")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")


def open_serial(port, baud, reset=False):
    """Open port without letting DTR/RTS reset the ESP32 or put it in bootloader mode."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 2
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    if reset:
        ser.rts = True          # pull EN low
        time.sleep(0.1)
        ser.rts = False
        time.sleep(1.0)         # let the camera initialise
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


class JpegStream:
    """Extracts complete JPEGs from a raw byte stream and always returns the newest."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.total_bytes = 0

    def _extract_latest(self):
        latest = None
        while True:
            start = self.buf.find(SOI)
            if start < 0:
                if len(self.buf) > 1:      # keep last byte, it might be the 0xFF of a SOI
                    del self.buf[:-1]
                break
            if start > 0:
                del self.buf[:start]
            end = self.buf.find(EOI, 2)
            if end < 0:
                if len(self.buf) > MAX_FRAME_BYTES:   # runaway frame, resync
                    del self.buf[:2]
                    continue
                break
            latest = bytes(self.buf[:end + 2])
            del self.buf[:end + 2]
        return latest

    def _fill(self):
        chunk = self.ser.read(max(1, self.ser.in_waiting))
        if not chunk:
            return False
        self.total_bytes += len(chunk)
        self.buf.extend(chunk)
        if len(self.buf) > MAX_BUFFER:
            del self.buf[:-MAX_FRAME_BYTES]
        return True

    def read_latest(self):
        """Block until a full JPEG arrives; return the newest one. None on timeout."""
        while True:
            frame = self._extract_latest()
            if frame is not None:
                break
            if not self._fill():
                return None
        # Drop stale frames if we are slower than the camera (e.g. YOLO on CPU)
        while self.ser.in_waiting:
            self._fill()
            newer = self._extract_latest()
            if newer is not None:
                frame = newer
        return frame


def load_model(path):
    from ultralytics import YOLO

    if not os.path.isfile(path):
        print(f"[WARN] ไม่พบโมเดล '{path}' -> ใช้ yolov8n.pt แทน (จะดาวน์โหลดอัตโนมัติ)")
        path = "yolov8n.pt"
    print(f"Loading YOLO model: {path}")
    return YOLO(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="เช่น /dev/ttyUSB0 หรือ COM3")
    parser.add_argument("--baud", type=int, default=460800, help="ต้องตรงกับ SERIAL_BAUD ใน firmware")
    parser.add_argument("--model", default="models/best.pt")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--no-yolo", action="store_true", help="แสดงภาพจากกล้องอย่างเดียว (ทดสอบกล้อง)")
    parser.add_argument("--reset", action="store_true", help="รีเซ็ต ESP32 ผ่าน RTS ตอนเปิดพอร์ต")
    args = parser.parse_args()

    print("Available serial ports:")
    list_ports()

    model = None if args.no_yolo else load_model(args.model)

    print(f"Opening {args.port} at {args.baud} baud (Ctrl+C to stop, q to quit window)")
    try:
        ser = open_serial(args.port, args.baud, args.reset)
    except serial.SerialException as e:
        print(f"[ERROR] เปิดพอร์ตไม่ได้: {e}")
        print("  - ตรวจชื่อพอร์ตด้วย --port (ดูรายการด้านบน)")
        print("  - Linux: sudo usermod -aG dialout $USER แล้ว logout/login ใหม่")
        print("  - ปิดโปรแกรมที่ใช้พอร์ตอยู่ (Arduino Serial Monitor ฯลฯ)")
        sys.exit(1)

    show = not args.no_display
    frame_count = 0
    last_report = time.monotonic()
    fps = 0.0
    waits = 0

    try:
        with ser:
            stream = JpegStream(ser)
            while True:
                jpeg = stream.read_latest()
                if jpeg is None:
                    waits += 1
                    if stream.total_bytes == 0:
                        print(f"[WAIT] {waits * 2}s: ไม่ได้รับข้อมูลเลย - กดปุ่ม RESET บนบอร์ด, "
                              f"ลอง --reset, เช็คว่า GPIO0 ไม่ได้ต่อ GND, และกล้อง init ผ่าน")
                    else:
                        print(f"[WAIT] {waits * 2}s: รับข้อมูลแล้ว {stream.total_bytes} bytes "
                              f"แต่ไม่พบ JPEG - baud น่าจะไม่ตรงกับ firmware (ตอนนี้ {args.baud})")
                    continue
                waits = 0

                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue  # corrupted frame, wait for the next one

                result = None
                if model is not None:
                    result = model.predict(image, conf=args.conf, imgsz=args.imgsz,
                                           device="cpu", verbose=False)[0]
                    annotated = result.plot()
                else:
                    annotated = image

                frame_count += 1
                now = time.monotonic()
                if now - last_report >= 1.0:
                    fps = frame_count / (now - last_report)
                    frame_count = 0
                    last_report = now

                label = f"{'YOLO' if model else 'RAW'} {fps:.1f} FPS | {image.shape[1]}x{image.shape[0]}"
                cv2.putText(annotated, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2, cv2.LINE_AA)

                if show:
                    try:
                        cv2.imshow("ESP32-CAM Serial + YOLO (CPU)", annotated)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
                    except cv2.error as e:
                        print(f"[WARN] เปิดหน้าต่างแสดงภาพไม่ได้ ({str(e).strip()[:80]})")
                        print("       ลอง: pip uninstall opencv-python-headless && pip install opencv-python")
                        print("       ตอนนี้สลับเป็นโหมดไม่แสดงภาพ")
                        show = False
                else:
                    names = model.names if model else {}
                    dets = []
                    if result is not None and result.boxes is not None:
                        for box in result.boxes:
                            cid = int(box.cls[0])
                            dets.append(f"{names.get(cid, cid)}:{float(box.conf[0]):.2f}")
                    print(f"frame {fps:.1f} fps | " + (", ".join(dets) if dets else "no detections"))
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()