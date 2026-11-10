#!/usr/bin/env python3
"""Web dashboard backend for the arm: item list, pick queue, live camera, stop.

Serves the built Svelte dashboard (dashboard/dist) and a small JSON API. One worker
thread runs the queued picks one at a time with visual_pick.py's auto mode (pick, carry
to ArUco id 10, drop); after the queue empties the arm goes to the rest pose. The camera
is opened once and shared by the picks and the live stream.

Usage (run with the .venv-cpu interpreter, from python/):
  python armbot_server.py                 # dry run: real camera, no servo output
  python armbot_server.py --execute       # drive the real servos (asks first)
  python armbot_server.py --http          # plain HTTP; phones then cannot use mic or camera

Open https://<pi address>:8443 . Phones need HTTPS for the microphone and camera, so a
self-signed certificate is made on first start; accept the browser warning once.

API: GET /api/items, GET /api/status, POST /api/queue {"item_id"|"barcode"|"text"},
DELETE /api/queue/<id>, POST /api/stop, GET /stream.mjpg, GET /mqtt.json.

MQTT (calibration/mqtt.json, skipped with --no-mqtt), with <p> the prefix ("armbot"):
  published, retained:  <p>/online "1"/"0", <p>/items, <p>/status (same JSON as the API)
  published:            <p>/event {"type": "queued"|"unknown", ...} after each queue command,
                        <p>/camera JPEG 320x240 every camera_every_s (default 1 s; 0 = off)
  subscribed:           <p>/cmd/queue {"item_id"|"barcode"|"text", "source"},
                        <p>/cmd/cancel {"id"}, <p>/cmd/stop {}

Safety: anyone who can reach the port can move the arm. Stop (button or POST /api/stop)
ends the current pick at the next camera look and sends the arm to the rest pose, the
same as Ctrl+C in visual_pick.py.
"""

import argparse
import collections
import copy
import json
import mimetypes
import os
import signal
import ssl
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import visual_pick as vp  # noqa: E402
from pick_and_place import DryRunQuiet, open_camera  # noqa: E402
from servo_control_app import Pca9685  # noqa: E402

INVENTORY = ROOT / "calibration" / "inventory.json"
MQTT_CONFIG = ROOT / "calibration" / "mqtt.json"
DIST = ROOT / "dashboard" / "dist"
CERT_DIR = ROOT / "python" / ".armbot_certs"
STREAM_FPS = 8
ANNOTATED_FRESH_S = 1.0  # show the pick overlay while picks keep publishing it
LOG_LINES = 200


class LogTee:
    """Copies everything printed to the terminal into a buffer the dashboard can show."""

    def __init__(self, out):
        self.out = out
        self.lines = collections.deque(maxlen=LOG_LINES)
        self.partial = ""
        self.lock = threading.Lock()

    def write(self, text):
        self.out.write(text)
        with self.lock:
            self.partial += text
            *done, self.partial = self.partial.split("\n")
            self.lines.extend(line for line in done if line.strip())

    def flush(self):
        self.out.flush()

    def tail(self, n=40):
        with self.lock:
            return list(self.lines)[-n:]


class CameraHub:
    """One camera reader thread; picks get fresh frames, the stream gets the newest JPEG."""

    def __init__(self, camera_cfg, rotate=False):
        self.cam = open_camera(camera_cfg)
        self.rotate = rotate  # same 180 degree turn the picks apply (directions.image_upright false)
        self.cond = threading.Condition()
        self.frame, self.seq = None, 0
        self.annotated, self.annotated_at = None, 0.0
        self.stop_event = threading.Event()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            try:
                frame = self.cam.read()
            except RuntimeError:
                continue
            with self.cond:
                self.frame, self.seq = frame, self.seq + 1
                self.cond.notify_all()

    def read(self):
        """Wait for a frame newer than the call, so a pick never acts on a stale one."""
        with self.cond:
            wanted = self.seq + 1
            if not self.cond.wait_for(lambda: self.seq >= wanted, timeout=5):
                raise RuntimeError("no camera frame")
            return self.frame.copy()

    def publish(self, frame):
        with self.cond:
            self.annotated, self.annotated_at = frame, time.monotonic()

    def stop_requested(self):
        return self.stop_event.is_set()

    def jpeg(self, size=None, quality=75):
        import cv2

        with self.cond:
            fresh = self.annotated is not None and time.monotonic() - self.annotated_at < ANNOTATED_FRESH_S
            frame = self.annotated if fresh else self.frame
        if frame is None:
            return None
        if self.rotate and not fresh:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        if size:
            frame = cv2.resize(frame, size)
        return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()

    def close(self):
        """Detectors call this when a pick ends; the shared camera stays open."""

    def shutdown(self):
        self.running = False
        self.cam.close()


class Inventory:
    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.data = json.loads(path.read_text(encoding="utf-8"))

    def items(self):
        with self.lock:
            return copy.deepcopy(self.data["items"])

    def find(self, item_id=None, barcode=None, text=None):
        with self.lock:
            for item in self.data["items"]:
                if item_id and item["id"] == item_id:
                    return copy.deepcopy(item)
                if barcode and item["barcode"] == barcode.strip():
                    return copy.deepcopy(item)
                if text and any(alias.lower() in text.lower() for alias in item["aliases"]):
                    return copy.deepcopy(item)
        return None

    def record_pick(self, item_id):
        with self.lock:
            for item in self.data["items"]:
                if item["id"] == item_id:
                    item["stock"] = max(0, item["stock"] - 1)
                    item["picks"] += 1
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            os.replace(tmp, self.path)


class Robot:
    """Queue plus the worker thread that runs one pick at a time."""

    def __init__(self, execute, inventory, hub):
        self.execute = execute
        self.inventory = inventory
        self.hub = hub
        self.main_cfg = vp.load_config(vp.DEFAULT_CONFIG)
        self.vcfg = vp.load_config(vp.VISUAL_CONFIG)
        self.lock = threading.Lock()
        self.jobs = []
        self.next_id = 1
        self.state = "idle"
        self.model = None
        hw = self.main_cfg["hardware"]
        self.pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True) if execute else DryRunQuiet()
        self.wake = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()

    def add(self, item, source):
        with self.lock:
            job = {"id": self.next_id, "item_id": item["id"], "name": item["name"], "name_th": item["name_th"],
                   "source": source, "state": "queued", "message": "", "created": time.time()}
            self.next_id += 1
            self.jobs.append(job)
        self.wake.set()
        return job

    def cancel(self, job_id):
        with self.lock:
            for job in self.jobs:
                if job["id"] == job_id and job["state"] == "queued":
                    job["state"] = "cancelled"
                    return True
        return False

    def stop(self):
        with self.lock:
            for job in self.jobs:
                if job["state"] == "queued":
                    job["state"] = "cancelled"
            busy = self.state == "picking"
        if busy:
            self.hub.stop_event.set()
        return busy

    def status(self):
        with self.lock:
            recent = [j for j in self.jobs if j["state"] in ("queued", "running")] + \
                     [j for j in self.jobs if j["state"] not in ("queued", "running")][-15:]
            return {"mode": "execute" if self.execute else "dry-run", "state": self.state,
                    "jobs": copy.deepcopy(recent)}

    def _next(self):
        with self.lock:
            for job in self.jobs:
                if job["state"] == "queued":
                    job["state"] = "running"
                    return job
        return None

    def _worker(self):
        while True:
            job = self._next()
            if job is None:
                if self.state == "picking":
                    self._rest()
                self.wake.wait(timeout=1)
                self.wake.clear()
                continue
            self.state = "picking"
            self.hub.stop_event.clear()
            ok, message = self._pick(job)
            with self.lock:
                job["state"], job["message"] = ("done" if ok else "failed"), message
            if ok:
                self.inventory.record_pick(job["item_id"])

    def _pick(self, job):
        item = self.inventory.find(item_id=job["item_id"])
        print(f"== Job {job['id']}: {item['name']}")
        vcfg = copy.deepcopy(self.vcfg)
        vp.apply_object_settings(vcfg, item["yolo_class"])
        try:
            if not item["color_only"] and self.model is None:
                import torch
                from ultralytics import YOLO

                torch.set_num_threads(3)
                self.model = YOLO(str(vp.rel(self.main_cfg["yolo"]["weights"])))
            detector = vp.Detector(self.main_cfg, vcfg, item["yolo_class"], False, item["color_only"],
                                   self.execute, cam=self.hub, model=self.model)
            arm = vp.Arm(self.pca, self.main_cfg, vcfg, dry_run=not self.execute)
            picks = vp.auto_run(arm, detector, vcfg, max_picks=1, wait_for_hand=False)
        except vp.StopRequested:
            print("Stopped from the dashboard. Going to the rest pose.")
            self._rest()
            return False, "stopped"
        except SystemExit as e:
            print(f"Pick refused: {e}")
            return False, str(e)
        except Exception as e:  # keep the server alive; the message shows on the dashboard
            print(f"Pick failed: {type(e).__name__}: {e}")
            return False, f"{type(e).__name__}: {e}"
        return (picks == 1), ("picked and placed" if picks == 1 else "not found")

    def _rest(self):
        self.state = "resting"
        try:
            vp.Arm(self.pca, self.main_cfg, copy.deepcopy(self.vcfg), dry_run=not self.execute).go_rest()
            print("At rest.")
        except Exception as e:
            print(f"Rest failed: {type(e).__name__}: {e}")
        self.state = "idle"


class MqttBridge:
    """Mirrors the queue and status on an MQTT broker and takes commands from it."""

    def __init__(self, cfg, robot, inventory, log, hub=None):
        import paho.mqtt.client as mqtt

        self.robot, self.inventory, self.log, self.hub = robot, inventory, log, hub
        self.camera_every_s = cfg.get("camera_every_s", 1.0)  # 0 = do not send snapshots
        self.p = cfg["prefix"]
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"armbot-{os.uname().nodename}")
        if cfg.get("username"):  # public brokers such as broker.hivemq.com take no login
            self.client.username_pw_set(cfg["username"], cfg.get("password"))
        self.client.will_set(f"{self.p}/online", "0", qos=1, retain=True)
        self.client.on_connect = self._connected
        self.closing = False
        self.client.on_disconnect = lambda *a: self.closing or print("MQTT disconnected; retrying.")
        self.client.on_message = self._message
        self.client.reconnect_delay_set(1, 30)
        self.client.connect_async(cfg["host"], cfg["port"], keepalive=30)
        self.client.loop_start()
        threading.Thread(target=self._publisher, daemon=True).start()

    def _send(self, topic, body, retain=False):
        self.client.publish(f"{self.p}/{topic}", json.dumps(body, ensure_ascii=False), qos=1, retain=retain)

    def _connected(self, client, _userdata, _flags, reason, _props):
        if reason.is_failure:
            print(f"MQTT connection refused: {reason}")
            return
        print(f"MQTT connected; topics {self.p}/#")
        client.subscribe(f"{self.p}/cmd/#", qos=1)
        client.publish(f"{self.p}/online", "1", qos=1, retain=True)
        self._send("items", {"items": self.inventory.items()}, retain=True)

    def _message(self, _client, _userdata, msg):
        try:
            body = json.loads(msg.payload or b"{}")
        except json.JSONDecodeError:
            body = {}
        cmd = msg.topic.removeprefix(f"{self.p}/cmd/")
        if cmd == "queue":
            item = self.inventory.find(item_id=body.get("item_id"), barcode=body.get("barcode"), text=body.get("text"))
            if item is None:
                self._send("event", {"type": "unknown", "query": body})
            else:
                self._send("event", {"type": "queued", "job": self.robot.add(item, body.get("source", "mqtt")),
                                     "item": item})
        elif cmd == "cancel":
            self.robot.cancel(int(body.get("id", 0)))
        elif cmd == "stop":
            self.robot.stop()

    def _publisher(self):
        last_status = last_items = None
        last_camera = 0.0
        while True:
            if self.hub and self.camera_every_s and self.client.is_connected() \
                    and time.monotonic() - last_camera >= self.camera_every_s:
                jpg = self.hub.jpeg(size=(320, 240), quality=60)  # about 10 KB, for the cloud dashboard
                if jpg:
                    self.client.publish(f"{self.p}/camera", jpg, qos=0)
                last_camera = time.monotonic()
            status = json.dumps({**self.robot.status(), "log": self.log.tail()}, ensure_ascii=False)
            items = json.dumps({"items": self.inventory.items()}, ensure_ascii=False)
            if self.client.is_connected():
                if status != last_status:
                    self.client.publish(f"{self.p}/status", status, qos=0, retain=True)
                    last_status = status
                if items != last_items:
                    self.client.publish(f"{self.p}/items", items, qos=1, retain=True)
                    last_items = items
            time.sleep(1)

    def close(self):
        self.closing = True
        self.client.publish(f"{self.p}/online", "0", qos=1, retain=True).wait_for_publish(2)
        self.client.loop_stop()
        self.client.disconnect()


class QuietServer(ThreadingHTTPServer):
    """Browsers drop TLS connections mid-way (certificate prompts, closed tabs); that is not an error."""

    daemon_threads = True

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], (ssl.SSLError, ConnectionResetError, BrokenPipeError, TimeoutError)):
            return
        super().handle_error(request, client_address)


def make_handler(robot, inventory, hub, log, mqtt_cfg=None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass  # keep the terminal for robot messages

        def _json(self, body, code=200):
            data = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            length = int(self.headers.get("Content-Length") or 0)
            try:
                return json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                return {}

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/api/items":
                return self._json({"items": inventory.items()})
            if path == "/api/status":
                return self._json({**robot.status(), "log": log.tail()})
            if path == "/stream.mjpg":
                return self._stream()
            if path == "/mqtt.json":
                if not mqtt_cfg:
                    return self._json({"error": "MQTT off"}, 404)
                return self._json({k: mqtt_cfg[k] for k in ("ws_url", "username", "password", "prefix")})
            self._static(path)

        def do_POST(self):
            path = self.path.split("?")[0]
            if path == "/api/queue":
                body = self._body()
                item = inventory.find(item_id=body.get("item_id"), barcode=body.get("barcode"),
                                      text=body.get("text"))
                if item is None:
                    return self._json({"error": "unknown item", "query": body}, 404)
                return self._json({"job": robot.add(item, body.get("source", "click")), "item": item})
            if path == "/api/stop":
                return self._json({"stopping": robot.stop()})
            self._json({"error": "not found"}, 404)

        def do_DELETE(self):
            parts = self.path.split("?")[0].strip("/").split("/")
            if len(parts) == 3 and parts[:2] == ["api", "queue"] and parts[2].isdigit():
                return self._json({"cancelled": robot.cancel(int(parts[2]))})
            self._json({"error": "not found"}, 404)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    jpg = hub.jpeg()
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(1 / STREAM_FPS)
            except (BrokenPipeError, ConnectionResetError, ssl.SSLError):
                pass

        def _static(self, path):
            target = (DIST / path.lstrip("/")).resolve()
            if path == "/" or not target.is_file() or DIST.resolve() not in target.parents:
                target = DIST / "index.html"  # single-page app
            if not target.is_file():
                return self._json({"error": "dashboard not built; run npm run build in dashboard/"}, 503)
            data = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(target.name)[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    return Handler


def ensure_certificate():
    """Self-signed certificate for this Pi's name and addresses, made once."""
    cert, key = CERT_DIR / "cert.pem", CERT_DIR / "key.pem"
    if cert.exists() and key.exists():
        return cert, key
    CERT_DIR.mkdir(exist_ok=True)
    host = os.uname().nodename
    ips = subprocess.run(["hostname", "-I"], capture_output=True, text=True).stdout.split()
    san = ",".join([f"DNS:{host}", f"DNS:{host}.local", "DNS:localhost", "IP:127.0.0.1"] + [f"IP:{ip}" for ip in ips])
    subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "825",
                    "-keyout", str(key), "-out", str(cert), "-subj", f"/CN={host}",
                    "-addext", f"subjectAltName={san}"], check=True, capture_output=True)
    return cert, key


def main():
    ap = argparse.ArgumentParser(description="Armbot web dashboard server")
    ap.add_argument("--execute", action="store_true", help="drive the real servos (default: dry run)")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--http", action="store_true", help="plain HTTP instead of HTTPS")
    ap.add_argument("--camera", choices=("serial", "usb"), help="override camera.source in the config")
    ap.add_argument("--no-mqtt", action="store_true", help="do not connect to the MQTT broker")
    ap.add_argument("--yes", action="store_true", help="skip the --execute safety question (systemd service)")
    args = ap.parse_args()

    if args.execute and not args.yes and not vp.ask("Servo power on, area clear, hand on the arm? The dashboard will move it"):
        print("Cancelled.")
        return
    log = LogTee(sys.stdout)
    sys.stdout = log
    inventory = Inventory(INVENTORY)
    camera_cfg = vp.load_config(vp.DEFAULT_CONFIG)["camera"]
    camera_cfg["source"] = args.camera or camera_cfg["source"]
    hub = CameraHub(camera_cfg, rotate=not vp.load_config(vp.VISUAL_CONFIG)["directions"]["image_upright"])
    robot = Robot(args.execute, inventory, hub)
    mqtt_cfg = None if args.no_mqtt or not MQTT_CONFIG.exists() else json.loads(MQTT_CONFIG.read_text("utf-8"))
    bridge = MqttBridge(mqtt_cfg, robot, inventory, log, hub) if mqtt_cfg else None
    server = QuietServer(("0.0.0.0", args.port), make_handler(robot, inventory, hub, log, mqtt_cfg))
    scheme = "http"
    if not args.http:
        cert, key = ensure_certificate()
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        server.socket = ctx.wrap_socket(server.socket, server_side=True)
        scheme = "https"
    ips = subprocess.run(["hostname", "-I"], capture_output=True, text=True).stdout.split()
    print(f"Dashboard ({'EXECUTE' if args.execute else 'dry run'}): "
          + ", ".join(f"{scheme}://{ip}:{args.port}" for ip in ips + ["localhost"]))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down: stopping the queue and going to the rest pose.")
        robot.stop()
        deadline = time.monotonic() + 30
        while robot.state != "idle" and time.monotonic() < deadline:
            time.sleep(0.2)
        robot._rest()
    finally:
        server.server_close()
        hub.shutdown()
        if bridge:
            bridge.close()


if __name__ == "__main__":
    main()
