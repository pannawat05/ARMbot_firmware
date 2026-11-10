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

# Armbot MQTT Topics

## Pi ส่งออก (publish): หน้าเว็บ subscribe topic เหล่านี้

| Topic | Payload | QoS | Retained | ส่งเมื่อไร | รายละเอียด |
|---|---|---|---|---|---|
| `armbot/online` | ข้อความ `"1"` หรือ `"0"` | 1 | ใช่ | `"1"` ตอนต่อ broker ได้, `"0"` ตอนปิดโปรแกรมหรือหลุด (Last Will) | บอกว่า Pi ออนไลน์อยู่ไหม ถ้าเป็น `"0"` หน้าเว็บจะขึ้นแถบสีแดง "เชื่อมต่อหุ่นยนต์ไม่ได้" |
| `armbot/status` | JSON `{"mode", "state", "jobs":[...], "log":[...]}` | 0 | ใช่ | เช็กทุก 1 วินาที และส่งเฉพาะตอนข้อมูลเปลี่ยน | สถานะแขนกลกับคิวงาน (ดูฟิลด์ในตารางถัดไป) |
| `armbot/items` | JSON `{"items":[...]}` | 1 | ใช่ | ตอนต่อ broker ได้ และตอนรายการของเปลี่ยน เช่น stock ลดหลังหยิบสำเร็จ | รายการของจาก `calibration/inventory.json` มีฟิลด์ `id`, `name`, `name_th`, `zone`, `stock`, `barcode`, `aliases`, `picks` |
| `armbot/event` | JSON `{"type":"queued","job":{...},"item":{...}}` หรือ `{"type":"unknown","query":{...}}` | 1 | ไม่ | หลังได้รับ `cmd/queue` ทุกครั้ง | ตอบกลับคำสั่งหยิบ: `queued` แปลว่าเข้าคิวแล้ว, `unknown` แปลว่าหาของไม่เจอ |
| `armbot/camera` | ภาพ JPEG (binary) ขนาด 320×240 ประมาณ 10 KB | 0 | ไม่ | ทุก 1 วินาที | ภาพจากกล้องบนแขน สำหรับหน้า dashboard บน cloud |

## Pi รับเข้า (subscribe `armbot/cmd/#`): หน้าเว็บหรือเครื่องอื่นส่งมาสั่งงาน

| Topic | Payload ตัวอย่าง | ผลที่เกิดขึ้น |
|---|---|---|
| `armbot/cmd/queue` | `{"item_id":"relay","source":"voice"}` | เพิ่มงานหยิบเข้าคิว |
| | `{"barcode":"ARM-0002","source":"barcode"}` | หาของจากบาร์โค้ด |
| | `{"text":"เอารีเลย์","source":"mqtt"}` | หาของจากคำใน `aliases` |
| `armbot/cmd/cancel` | `{"id":3}` | ยกเลิกงานหมายเลข 3 ถ้างานนั้นกำลังหยิบอยู่ แขนจะหยุดแล้วกลับไปท่าพัก |
| `armbot/cmd/stop` | `{}` | หยุดทันทีและยกเลิกทั้งคิว แขนจะกลับไปท่าพัก (rest pose) |

> ใน `cmd/queue` ใส่แค่อย่างเดียวจาก `item_id`, `barcode` หรือ `text` ส่วน `source` ไม่ใส่ก็ได้ ค่าเริ่มต้นคือ `"mqtt"`

## ฟิลด์ใน `armbot/status`

| ฟิลด์ | ค่าที่เป็นไปได้ | ความหมาย |
|---|---|---|
| `mode` | `execute` / `dry-run` | `execute` ขยับแขนจริง, `dry-run` ไม่ส่งสัญญาณไป servo |
| `state` | `idle` / `picking` / `resting` | ว่างอยู่ / กำลังหยิบ / กำลังกลับท่าพัก |
| `jobs[].id` | ตัวเลข | หมายเลขงาน ใช้กับ `cmd/cancel` |
| `jobs[].item_id`, `name`, `name_th` | เช่น `relay`, `รีเลย์โมดูล` | ของที่สั่งหยิบ |
| `jobs[].source` | `voice` / `barcode` / `click` / `mqtt` | ช่องทางที่สั่งเข้ามา |
| `jobs[].state` | `queued` → `running` → `done` / `failed` / `cancelled` | สถานะของงานแต่ละงาน |
| `jobs[].message` | ข้อความ | เหตุผลเมื่องานไม่สำเร็จ หรือรายละเอียดผลงาน |
| `jobs[].created` | Unix time | เวลาที่สั่งงาน |
| `log` | รายการข้อความ | log ล่าสุดของโปรแกรมหยิบ |
