# Handoff: eye-in-hand pick-and-place arm (Raspberry Pi 5 + PCA9685)

Written 2026-11-07 at the end of a long working session, for a new chat to pick up from.
Project root: `/home/pi/Documents/yolo_xyz_ddqn_pca9685_project` on the Pi (`pi@armbot`).
There is no git repository; files on disk are the only history.

## 1. What the project is

A 6-servo hobby robot arm on a table. A camera is mounted on the gripper (eye-in-hand).
The goal is autonomous pick-and-place:

1. Find an object on the table (YOLO class, confirmed by colour), turn and reach toward it.
2. Grip it, lift it straight up about 5 cm.
3. Find the destination, an ArUco marker (DICT_4X4_50, id 10), with OpenCV. YOLO is not used for this.
4. Carry the object over the marker and release it.
5. Go back to the start pose and repeat until nothing is left.

This works end to end for the relay module and has completed full runs (log lines
`Placed. YOLO back on.` and `Pick 1 done.`). Accuracy and grip reliability are still being tuned (section 9).

The working program is `python/visual_pick.py`. The older `python/pick_and_place.py`
(fixed camera, homography, IKPy + DDQN) is not used for picking any more but still provides
the camera readers, ArUco helper and `--preview` / `--aruco-preview`.

### The user
- Writes in English or Thai; **answers must be in Thai** (saved in memory).
- Runs every `--execute` command themself, standing at the arm. The assistant must not drive the
  real servos itself; it runs dry runs, simulations, camera checks and register reads only.
- Wants autonomy: the arm should decide and move on its own, not stop and ask, except where safety needs it.
- Most recent request: a tuning guide they can edit themself (done, section 8).

## 2. Hardware

| Part | Detail |
|---|---|
| Computer | Raspberry Pi 5, Raspberry Pi OS (Debian 13), Python 3.13.5, kernel 6.18 |
| Servo driver | PCA9685 on I2C bus 1, address 0x40, 50 Hz |
| Servo power | 5 V 15 A supply on PCA9685 V+ (Pi powered separately). Never measured under load. |
| Camera now | Nubwo 2K USB webcam, Linux name `USB Camera` (Sunplus 1bcf:28c4), MJPG 640x480, about 10 fps, autofocus. On the gripper. |
| Camera before | ESP32-CAM (AI-Thinker, OV2640) over a CH340 USB-serial cable at 460800 baud on `/dev/ttyUSB0`. Still connected, can be selected with `--camera serial`. |
| Gripper pads | Hard plastic, no rubber. Objects slip. |

### Servo channels (PCA9685)

| Ch | Joint | Servo | Notes |
|---|---|---|---|
| 1 | Base (yaw) | MG996R | The only joint that turns the gripper left/right. |
| 2 | Shoulder | TD-8125MG (25 kg) | |
| 3 | Elbow | MG996R | |
| 4 | Wrist roll | SG90 | Not driven by visual_pick, held at 90 by Close & hold. |
| 5 | Wrist pitch | SG90 | Held at 160 (150 until 2026-11-13) so the camera looks at the table. |
| 6 | Gripper | SG90 | 50 = grip, 150 = open. |

All channel numbers are PCA9685 channels as wired (assumed from the user's list, confirmed by use).

### Geometry and directions
- All servos at 90 (except gripper) = an inverted L / Greek Gamma: upper arm vertical, forearm horizontal forward.
- Measured directions (`--direction-test`, stored in `calibration/visual_pick_config.json`):
  - shoulder: servo angle up = upper arm leans forward (`shoulder_forward_sign: 1`)
  - elbow: servo angle **down** = forearm down (`elbow_down_sign: -1`)
  - base: servo angle up = arm turns left seen from behind (`base_right_sign: -1`)
- Start / standby pose: base 90, shoulder 85, elbow 100 (elbow was 70, then 86 on 2026-11-10; shoulder 90 and elbow 86 until 2026-11-13, raised about 4 cm), wrist roll 90, wrist pitch 160.
  This is `STANDBY_DEG` in `servo_control_app.py` and `start_servo_deg` + `hold_servo_deg` in the config.
- Link lengths in `calibration/pick_place_config.json` (`arm` section) are **estimates from a drawing**,
  never measured: shoulder height 50.87, upper arm 113.12, forearm 97.05, tool 46.0 mm.
- Height correction: `tip_offset_mm: 36` was set from **one** ruler measurement (real tip height 80 mm
  where the model said 44 mm). Heights near 80 mm are fair; low heights (25 to 40 mm) are less certain.
- The model assumes the tool is tilted 70 degrees below the forearm line because wrist pitch is 160.
- Measured: camera lens to gripper centre = **8 cm** (user measured, Nubwo camera). Used as `gripper_offset_mm`.

## 3. Software layout

| File | Role |
|---|---|
| `python/visual_pick.py` | **Main program.** Eye-in-hand search, aim, descend, grip, lift, ArUco place, auto loop. Tuning guide in its module docstring. |
| `calibration/visual_pick_config.json` | **Main config** for visual_pick (poses, signs, detection, colour, approach, limits, grip, place). |
| `calibration/pick_place_config.json` | Shared config: camera, YOLO weights, servo channels/types/limits, gripper angles, I2C. |
| `python/pick_and_place.py` | Older fixed-camera pipeline. Provides `SerialCamera`, `UsbCamera`, `open_camera`, `aruco_detector`, `--preview`, `--aruco-preview`, `--calibrate-homography`. |
| `python/servo_control_app.py` | Tkinter manual servo app: 6 slots, +/- 10 degree steps, Grip/Ungrip, Limp, Standby, Close & hold, ALL OFF. Contains the `Pca9685` driver class. |
| `python/test_camera_yolo.py` | ESP32-CAM serial reader (`open_serial`, `JpegStream`). |
| `python/kuka_dynamics.py`, `ddqn.py`, `models/ddqn_6dof_arm_pick_place.zip` | Original DDQN simulation and checkpoint. Used only by pick_and_place.py. The checkpoint scored 0% success in the project's own evaluation. **Not used by visual_pick.** |
| `models/best.pt` | YOLO model (Ultralytics, classes `bananapi`, `relay_module`, `ultrasonic_hc-sr04`), trained 2026-10-29, MD5 `35d27c2d2f7d0e4f7e5aee679ccafdf7`. The user says this is their latest model. |
| `firmware/esp32cam_stream/` | PlatformIO project for the ESP32-CAM (460800 baud, JPEG Q12, VGA, grab latest, 2 ms gap after each frame). Flashed. |
| `firmware/esp32cam_backup_460800.bin` | Full 4 MB flash backup of the ESP32-CAM taken before reflashing. |
| `docs/aruco_markers/` | Printable ArUco markers (id 0-3 for homography, id 10 destination), 50 mm, PDF and PNGs. |
| `cpp/` | Original C++ PCA9685 daemon. Not used. |

Python environment: always run with `.venv-cpu/bin/python` from `python/`
(torch 2.6.0+debian CPU, ultralytics 8.4.183, OpenCV 4.10, ikpy, stable-baselines3, gymnasium, pyserial, smbus).
`.venv` is an older environment that is not used.

## 4. How visual_pick.py works

### Camera
`open_camera(cfg)` in pick_and_place.py returns `UsbCamera` when `camera.source` is `usb`
(current default) or `SerialCamera` for the ESP32-CAM. `--camera serial|usb` overrides.
- `UsbCamera` finds the device by name (`usb_name`), opens by index with V4L2, MJPG 640x480,
  buffer size 1, and calls `grab()` before `read()` only when the last read was more than 0.15 s ago
  (drops the stale buffered frame after a pause).
- `SerialCamera` opens the port with an EN reset pulse (a plain open often left the ESP32 silent),
  3 s first-frame timeout, 3 attempts, and flips the image left-right (`mirror: true`; the ESP32-CAM image is mirrored).

### Detection (`Detector`)
- `relay_module` (default, no `--color`): YOLO must say relay_module at conf >= 0.35 **and** a blue HSV blob
  must be present. The blob gives the position (YOLO boxes are often near full-frame and wrong).
  The colour check runs first (2 ms); YOLO (about 700 ms) runs only when there is a blob,
  and only until it has confirmed once per approach (`confirmed`), after which colour alone tracks it.
- `--color`: colour only, no YOLO. For `ultrasonic_hc-sr04` the colour entry also requires
  2 Hough circles (the transducers) on the blob; the position is their midpoint.
- Classes without a colour entry are blocked from `--execute` unless `--trust-yolo`.
- Marker: `find_marker` uses OpenCV ArUco DICT_4X4_50, id from `aruco.tray_marker_id` (10).
- `torch.set_num_threads(3)` (the default here was 1; 3 measured fastest).

### Arm model (`Arm`)
- Poses are offsets in degrees from the neutral (all 90) pose: base right, shoulder forward, elbow down are positive.
- `tip_height(pose)` and `reach(pose)` are a 2-D forward model of the estimated links plus `tip_offset_mm`.
- `allowed()` clips to `limits` and refuses lowering below `min_tip_height_mm`.
- `ramp()` interpolates at `ramp_deg_per_s` (20) or `carry_deg_per_s` (10) while holding an object.
- `move_tip(height, reach)` moves the tip in a straight line using a numeric Jacobian on shoulder + elbow.
- `go_start()` sends the start pose joint by joint (jumps if the arm was elsewhere and unpowered).
- `go_standby()` reads the real PCA9685 pulses and ramps back to the start pose from there.

### Pick sequence (`--auto`)
1. `go_start`, open gripper (150).
2. **Search**: sweep shoulder and elbow down 2 degrees per look (settle 0.4 s), restart from the top at the
   bottom (`search_min_tip_mm` 86). Stops after `auto.max_sweeps` (2) full sweeps with nothing seen.
3. **Aim** (`aim_before_pick: true`): turn the base so the object is at `target_u_frac` 0.5; slide the camera
   out/back **without tilting** (shoulder and elbow by equal and opposite amounts) until it is at
   `target_v_frac` 0.5, within `aim_deadband` 0.05. The object is then under the lens.
4. After `commit_detections` (1) sighting there, **descend blind**: one residual base turn, compute
   `reach + gripper_offset_mm` (80), `move_tip` to `align_tip_mm` (80) at that reach, then straight down to
   `grip_tip_mm` (38). Prints `Stopped short` if a joint limit stops it.
5. Close gripper to `grip_deg` (50) and keep PWM on; lift straight up `lift_mm` (50).
6. **Place**: "YOLO off", scan for marker id 10 from the start pose across `scan_base_deg`
   (0, -15, 15, -30, 30, -45, 45) in `scan_rows` (0, 8), aim at it the same way (staying above
   `carry_tip_mm` 85), move out by `gripper_offset_mm` at `drop_tip_mm` 80, open the gripper.
   If aiming does not finish it places from the best aim. If the marker is never found and was not seen
   earlier in the run, it holds the object and waits for Enter.
7. Back to the start pose, repeat.

### Other modes
| Command | What it does |
|---|---|
| `--detect` | Everything up to grip height, then stops without gripping. Use for alignment checks. |
| no flag (object only) | One pick, lift, then waits for Enter to open (no marker). |
| `--standby` | Ramp back to the start pose from the real servo pulses. |
| `--grip-test` | Gripper holds a hand-placed object through still / base turn / reach-out moves. No camera. |
| `--direction-test` | Nudges each joint 8 degrees and asks which way it went; writes the signs. |
| `--max-picks N` | With `--auto`, stop after N picks. |
| `CLASS --learn-color` | Samples the colour in the middle of the camera view and saves it as `color.classes.CLASS` after a y/n. Added for `bananapi`; once saved, YOLO picks of that class are colour-confirmed and `--execute` is allowed. Not yet run on the real board (camera was unplugged). |
| without `--execute` | Dry run: real camera, no servo output (`DryRunQuiet`). Poses are simulated, so the base often runs to its limit; that is expected. |

### Commands the user runs
```bash
cd /home/pi/Documents/yolo_xyz_ddqn_pca9685_project/python
../.venv-cpu/bin/python visual_pick.py --standby
../.venv-cpu/bin/python visual_pick.py relay_module --detect --show --execute
../.venv-cpu/bin/python visual_pick.py relay_module --auto --max-picks 1 --show --execute
../.venv-cpu/bin/python visual_pick.py relay_module --auto --show --execute
../.venv-cpu/bin/python visual_pick.py ultrasonic_hc-sr04 --color --auto --max-picks 1 --show --execute
../.venv-cpu/bin/python pick_and_place.py --aruco-preview
python3 servo_control_app.py
```
`--execute` asks a y/n question in the terminal first.

## 5. Safety behaviour (keep it)
- Hobby servos have no position feedback. The first command jumps to the start pose unless the arm is already
  there and powered. Recommended start: servo app, Standby, then **Close & hold** (exits while keeping PWM on slots 1-5).
- `Pca9685(..., keep_outputs=True)` does not reset the chip if it is already running, so PWM held by another
  program is not cut on startup. The servo app itself still calls `all_off` when it opens, so the arm drops then.
- Rest pose (added 2026-11-07, `rest_servo_deg`): servo angles base 90, shoulder 70, elbow 140, wrist roll 90,
  wrist pitch 110, gripper 150 (arm raised skyward). After a pick run (not `--detect`), and on Ctrl+C, `q`,
  SIGTERM or SIGQUIT, the arm ramps there, opens the gripper (dropping anything held), waits 1 s and cuts PWM to all servos (limp). A second Ctrl+C
  during that move holds still. SIGHUP is not handled (it was swallowed in tests); SIGKILL/SIGSTOP cannot be.
  `--rest` goes there on its own; `go_start` now ramps from the real pulses instead of jumping.
- The closed gripper keeps PWM on until release, in both visual_pick and the app (Grip holds; Ungrip opens and then relaxes).
- Every servo angle is checked against `joints.list[*].min_deg/max_deg` and the visual `limits`; a violation aborts.
- `hardware.cut_pwm_at_end: false`: the arm holds its pose after a run.

## 6. History of findings (evidence, so they are not re-learned)

| Finding | Evidence |
|---|---|
| The ESP32-CAM image is left-right mirrored. | User saw it; after flipping, YOLO changed from bananapi (wrong) to relay_module on the same frame; ArUco cannot read mirrored markers. |
| ESP32-CAM serial link is saturated. | 46.1 KB/s = 100% of 460800 baud, 4.7 fps, 9.8 KB frames. 576000 / 691200 / 921600 baud corrupted 12% / 94% / 78% of frames on this CH340. |
| ESP32-CAM often stayed silent after a plain port open. | 3 of 4 plain opens gave about 400 bytes then nothing; EN reset on open fixed it (10/10 opens in 1.5-2.6 s after reflash). |
| The Nubwo USB camera is much better. | 0.8 s open, about 10 fps, sharpness (Laplacian variance) 214 versus 3-16 for the ESP32-CAM. |
| YOLO on CPU is the remaining delay. | About 700 ms per 640 px frame (520 at 480, 380 at 320). Camera read about 100 ms, colour 2 ms, ArUco 18 ms. Pi not throttled. |
| `models/best.pt` gives false positives on this camera. | relay_module 0.70-0.85 on a tool case and on empty table; bananapi 0.4-0.58 on ceiling and table; HC-SR04 in clear view scored 0.09 as ultrasonic and 0.83 as relay_module. Hence the colour confirmation. |
| imgsz 320 makes false positives worse. | bananapi 0.72 with a full-frame box at 320. |
| The jaws are behind the camera. | Photo: camera over the relay, jaws about 8-9 cm toward the base. Now `gripper_offset_mm` 80 (user measurement for the Nubwo mount). |
| Leaning the shoulder alone tilts the camera and pushes the target off the top of the image. | Marker run: v went 0.36 to 0.20 and the marker was lost. Fixed by equal and opposite shoulder/elbow moves. |
| Elbow-only lift drags the object along the table. | Elbow-only lift moved the tip about 51 mm outward. Fixed by a straight-up `move_tip` lift. |
| Shoulder limit decides how low the tip can go far out. | At 150 degrees the tip stopped at about 50 mm far out; limit raised to 160 (visual `shoulder_forward_max_deg` 70 and joint `max_deg` 160). |
| Base sensitivity. | About 40 degrees of base per full image width (10.8 degrees moved the object 0.28 of the width). |
| Gripper strength itself is probably fine. | The app's Grip held well; `--grip-test` reportedly passed ("It works"). Grip angle was set back from 35 to 50. Slips in auto runs are more likely from where the jaws close (too low, misaligned) than from torque. |

## 7. Current configuration values (the ones that matter)

`calibration/visual_pick_config.json`:
- `approach`: target_u_frac 0.5, target_v_frac 0.5, aim_deadband 0.05, aim_gain_deg 40, aim_turn_frac 0.6,
  base_deg_per_u 40, aim_before_pick true, commit_detections 1, max_aim_steps 40, search_settle_s 0.4,
  search_elbow/shoulder_step_deg 2, lost_timeout_s 6, ramp_deg_per_s 20, align_tip_mm 80,
  **gripper_offset_mm 80**, **grip_tip_mm 38**.
- `limits`: base_max 45, shoulder_forward_max 70, shoulder_back_max 10, elbow_down_max 60, elbow_up_max 60,
  min_tip_height_mm 30, search_min_tip_mm 86, tip_offset_mm 36.
- `grip`: stop_settle_s 0.5, lift_mm 50, carry_deg_per_s 10.
- `place`: scan_base_deg [0,-15,15,-30,30,-45,45], scan_rows [0,8], looks_per_stop 2, carry_tip_mm 85, drop_tip_mm 80.
- `detection`: imgsz 640, follow_confidence 0.35, max_box_area_frac 0.7.
- `color`: relay_module HSV 95-125 / 110-255 / 70-255, min_area 0.05; ultrasonic_hc-sr04 same HSV, min_area 0.02, circles 2.

`calibration/pick_place_config.json`:
- camera: source usb, usb_name "USB Camera", usb_mirror false (ArUco reads it, so not mirrored), serial port/baud kept.
- gripper: channel 6, SG90, open 150, grip 50, settle 0.8 s.
- joints: shoulder max_deg 160, others 30-150.

## 8. Tuning guide

The full table is the module docstring at the top of `python/visual_pick.py` (starts at the line
"Tuning guide."). Summary: jaws short toward the base -> raise `gripper_offset_mm`; jaws to the left -> lower
`target_u_frac` in 0.01 steps; jaws hit the table -> raise `grip_tip_mm`; heights all off by the same amount ->
change `tip_offset_mm`. Keep `search_min_tip_mm >= align_tip_mm > grip_tip_mm > min_tip_height_mm`.
Always check with `--detect` first.

## 9. Open problems (current state)

1. **Jaws land about 0.6 cm short of the object's end.** The user said "about 0.6 cm south of the end of the object".
   The direction is not confirmed; if "south" means toward the base, raise `gripper_offset_mm` from 80 to about 86.
   Not applied: the user wants to tune it themself.
2. **Jaws hit the desk before reaching the object.** `grip_tip_mm` raised from 25 to 38 and the floor
   `min_tip_height_mm` from 15 to 30. **Not yet tested on the arm.** Low heights rely on estimated link lengths.
   The user also asked how to handle the arm going too low earlier in the sequence (answer: `search_min_tip_mm`,
   `align_tip_mm`, `grip_tip_mm`, `tip_offset_mm`, or the start elbow angle).
3. **Grip slipping.** Plastic jaws without rubber. Gripping too low (on the table) or off-centre makes it worse.
   Suggested hardware fixes: rubber pads, MG90S instead of SG90 for the gripper. Supply voltage at PCA9685 V+
   under load has never been measured (should stay above about 4.8 V).
4. **Perceived camera delay** is mostly YOLO on CPU (about 700 ms). Mitigated by colour gating; YOLO now runs
   only to confirm. `pick_and_place.py --preview` still runs YOLO every frame and looks laggy.
   Not tried: exporting the model to NCNN (likely much faster on the Pi; needs new packages and disk space).
5. **The YOLO model is unreliable on this camera** (false positives listed in section 6). Picking works only
   because colour confirms it. Proper fix: collect images from the gripper camera (with and without objects,
   including the green mat and the blue tool case) and retrain. bananapi has no colour entry, so it cannot be
   picked without `--trust-yolo`.
6. **HC-SR04 picking is untested on the arm.** Colour + 2-circle detection works on live frames and in dry runs.
   Grip height and jaw orientation were tuned for the relay; wrist roll is not driven, so the sensor must be
   placed with its long side across the jaw closing direction.
7. **Placing accuracy is unverified.** `gripper_offset_mm` is shared by picking and placing; the drop
   height 80 mm may make the object bounce. If the marker is covered by a placed object, later runs reuse
   its remembered position (only within one run).
8. **Geometry is estimated.** Link lengths unmeasured; one-point height calibration; tool pitch assumed
   60 degrees from wrist pitch 150. Measuring the links would improve every height and reach.
9. **Disk space is nearly full:** 98% used, about 577 MB free on `/` (29 GB card). Avoid large installs or
   pip downloads without cleaning first (pip once pulled about 1.6 GB of CUDA wheels; `.venv` is unused).
10. The servo app's `+/-` step is 10 degrees, which is coarse for calibration.

## 10. Things to avoid
- Do not run `--execute` or otherwise move the servos without the user present and asking.
- Do not "fix" safety gates by setting flags to true.
- Do not install a CUDA torch; use `pip --no-deps` if anything must go into `.venv-cpu`.
- Do not edit `/home/pi/Documents/robot_vision_app` (a different project; the firmware was copied out of it).
- Closing the servo app with the window X calls ALL OFF and the arm drops; use Close & hold.
- Only one program can hold `/dev/video0` (e.g. Cheese blocks it) or `/dev/ttyUSB0`.

## 11. Suggested next steps
1. User tests `grip_tip_mm` 38 with `--detect`, measures tip-to-table gap and jaw offset, adjusts
   `grip_tip_mm`, `gripper_offset_mm` and `target_u_frac` using the tuning guide.
2. Add rubber pads to the jaws; rerun `--grip-test`, then `--auto --max-picks 1`.
3. Verify the drop over the marker and measure the placing error.
4. Retrain YOLO on images from the gripper camera; then consider NCNN export for speed.
5. Measure the arm link lengths and update `pick_place_config.json` `arm` to replace the estimates.

## Web dashboard (added 2026-11-10)
- `python/armbot_server.py`: stdlib HTTP(S) server on port 8443. One shared camera (`CameraHub`), one worker
  thread running queued picks with `visual_pick.auto_run(max_picks=1)`, rest pose when the queue empties,
  Stop button/`POST /api/stop`, Ctrl+C/SIGTERM rest. Dry run unless `--execute`. Self-signed cert in
  `python/.armbot_certs/` (phones need HTTPS for mic and camera). `--http`, `--port`, `--camera serial|usb`.
- `calibration/inventory.json`: items (id, names, yolo_class, color_only, zone, stock, barcode, voice aliases,
  picks). Zones, stock and barcodes are examples. Stock/picks update after successful picks.
- `dashboard/`: Svelte 5 + Vite. Mobile: camera + voice on top, scan button + item cards below. Desktop: camera,
  voice, queue and log on the left; filterable table and search/USB-scanner input on the right. Voice uses the
  browser Web Speech API (th-TH, needs internet; "Hey Armbot" wake phrase, reply via speechSynthesis).
  Barcode by phone uses BarcodeDetector (Chrome Android). Build: `dashboard/build-with-docker.sh` (Docker is
  used only to build; the server runs on the Pi). Not tested yet with a real camera, phone or the arm.
- MQTT (2026-11-10): `calibration/mqtt.json` (git-ignored; holds the broker password) points to
  `mqtt.panplay-itgoeasy.xyz`, TCP 1883 for the Pi, `ws://...:9001` for the browser, prefix `armbot`.
  The server publishes retained `armbot/online|items|status`, `armbot/event`, and takes `armbot/cmd/queue|cancel|stop`
  (tested against the broker under a test prefix). The page uses MQTT only over http:// and falls back to the
  Pi's HTTP API after 5 s; over https:// it always uses HTTP (ws:// is blocked from https). On 2026-11-10 port 9001
  was not reachable from the Pi (TCP refused/filtered), so browsers fall back to HTTP until the broker exposes it.
