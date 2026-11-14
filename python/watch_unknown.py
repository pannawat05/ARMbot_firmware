#!/usr/bin/env python3
"""Watch the "Unknown object detected" MQTT topics and save the pictures.

  python watch_unknown.py                  # prefix and broker from calibration/mqtt.json
  python watch_unknown.py --out ~/unknown  # where the pictures go (default: ./unknown_seen)
  python watch_unknown.py --prefix armbot-010eb21b --host broker.hivemq.com

Topics, with <p> the prefix:
  <p>/unknown         JSON: message, job_id, box (pixels), box_center_size (YOLO, fractions), file
  <p>/unknown/image   JPEG with the box drawn, saved here as unknown-<job>-<time>.jpg
Ctrl+C to stop. Needs only paho-mqtt (already in .venv-cpu).
"""

import argparse
import json
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description="Print unknown-object messages and save their pictures")
    ap.add_argument("--prefix", help="MQTT topic prefix (default: from calibration/mqtt.json)")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int)
    ap.add_argument("--out", default="unknown_seen", help="folder for the pictures")
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)  # show lines at once even when piped

    cfg = {}
    cfg_path = ROOT / "calibration" / "mqtt.json"
    if cfg_path.exists():
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    prefix = args.prefix or cfg.get("prefix", "armbot")
    host = args.host or cfg.get("host", "broker.hivemq.com")
    port = args.port or cfg.get("port", 1883)
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    last = {"job": "x"}

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"watch-unknown-{int(time.time())}")
    if cfg.get("username") and not args.host:
        client.username_pw_set(cfg["username"], cfg.get("password"))

    def on_connect(c, *_):
        c.subscribe([(f"{prefix}/unknown", 1), (f"{prefix}/unknown/image", 0)])
        print(f"Connected to {host}:{port}. Watching {prefix}/unknown and {prefix}/unknown/image ...")

    def on_message(_c, _u, msg):
        if msg.topic.endswith("/unknown/image"):
            path = out / f"unknown-job{last['job']}-{time.strftime('%Y%m%d-%H%M%S')}.jpg"
            path.write_bytes(msg.payload)
            print(f"  picture saved: {path} ({len(msg.payload)} bytes)")
            return
        info = json.loads(msg.payload)
        last["job"] = info.get("job_id", "x")
        print(f"{time.strftime('%H:%M:%S')}  {info.get('message')}  job {info.get('job_id')}")
        print(f"  box {info.get('box')}  centre/size {info.get('box_center_size')}  on the Pi: {info.get('file')}")

    client.on_connect, client.on_message = on_connect, on_message
    client.connect(host, port, keepalive=30)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
