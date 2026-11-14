#!/usr/bin/env python3
"""Sweep the arm's big servos (base MG996R, shoulder TD-8125MG, elbow MG996R) from 30 to 150 degrees.

One joint at a time, slowly, speeding up and slowing down smoothly, with a pause at each end.
After each joint the pulse the PCA9685 is really sending is read back and compared with what
was asked, so a servo that does not follow (or a dead channel) shows up.

Usage (run with the .venv-cpu interpreter, with the armbot service stopped):
  python servo_sweep_test.py                      # dry run: prints the plan, no servo output
  python servo_sweep_test.py --execute            # move the real arm (asks first)
  python servo_sweep_test.py --execute --joint shoulder --cycles 3 --speed 15
  python servo_sweep_test.py --execute --end limp # cut PWM on the tested joints at the end

Safety:
- Hobby servos give no position feedback. The first move ramps from the angle the chip is
  sending now (or from 90 if the channel is off, which makes that servo jump to 90).
- 30 to 150 degrees is a big swing for the shoulder and elbow: the arm can reach the table,
  its own base or the camera cable. Clear the area, hold the arm, and use --min and --max to
  test a smaller range first.
- The range is clipped to each joint's min_deg and max_deg in calibration/pick_place_config.json.
- Speed is limited to the servo's rated speed (SERVO_TYPES deg_per_s).
- Ctrl+C stops where it is and keeps holding that pose; the arm does not move again.
  Run servo_control_app.py or visual_pick.py --rest afterwards.
- Refuses to move the arm while the armbot service is running: two programs would fight.
"""

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from servo_control_app import (  # noqa: E402
    SERVO_TYPES, DryRunPca9685, Pca9685, angle_to_us, us_to_angle,
)

CONFIG = ROOT / "calibration" / "pick_place_config.json"
JOINTS = ("base", "shoulder", "elbow")
TICK_S = 0.02
CENTER_DEG = 90.0
READBACK_TOLERANCE_US = 6.0  # the chip works in steps of about 4.9 us at 50 Hz


class QuietDryRun(DryRunPca9685):
    """Dry run that remembers the last pulse per channel instead of printing every one."""

    def __init__(self):
        self.pulse = {}

    def set_pulse_us(self, ch, us):
        self.pulse[ch] = us

    def read_pulse_us(self, ch):
        return self.pulse.get(ch)

    def channel_off(self, ch):
        self.pulse.pop(ch, None)


def motion_profile(span, rate, accel):
    """Fractions of a move of span degrees, one per TICK_S: speed rises at accel up to rate,
    holds, then falls at the same accel. Short moves never reach rate."""
    if span <= 0:
        return [1.0]
    t_acc = rate / accel
    if span < rate * t_acc:
        t_acc = math.sqrt(span / accel)
        rate = accel * t_acc
    total = span / rate + t_acc

    def done(t):
        if t < t_acc:
            return 0.5 * accel * t * t
        if t > total - t_acc:
            return span - 0.5 * accel * (total - t) ** 2
        return 0.5 * accel * t_acc * t_acc + rate * (t - t_acc)

    n = max(1, math.ceil(total / TICK_S))
    return [min(1.0, done(i * total / n) / span) for i in range(1, n + 1)]


def move_time(span, rate, accel):
    return len(motion_profile(span, rate, accel)) * TICK_S


class Sweeper:
    def __init__(self, pca, joint, args, dry_run):
        self.pca, self.dry_run = pca, dry_run
        self.name, self.ch, self.type = joint["name"], joint["channel"], joint["type"]
        self.lo = max(args.min, joint["min_deg"])
        self.hi = min(args.max, joint["max_deg"])
        self.rate = min(args.speed, SERVO_TYPES[self.type]["deg_per_s"])
        self.accel = args.accel
        self.dwell = args.dwell
        self.angle = None  # where the servo is now, as far as we know
        self.mismatches = 0
        if (self.lo, self.hi) != (args.min, args.max):
            print(f"  {self.name}: range clipped to {self.lo:.0f}-{self.hi:.0f} deg (joint limits in the config).")
        if self.rate != args.speed:
            print(f"  {self.name}: speed limited to {self.rate:.0f} deg/s (rated speed of the {self.type}).")

    def send(self, deg):
        self.pca.set_pulse_us(self.ch, angle_to_us(self.type, deg))
        self.angle = deg

    def start_angle(self):
        us = self.pca.read_pulse_us(self.ch)
        if us is None:
            print(f"  {self.name}: channel {self.ch} is not sending a pulse; it will jump to {CENTER_DEG:.0f} deg first.")
            self.send(CENTER_DEG)
            self.sleep(0.8)
        else:
            self.angle = us_to_angle(self.type, us)
        return self.angle

    def sleep(self, s):
        if not self.dry_run:
            time.sleep(s)

    def ramp(self, target):
        start = self.angle
        for f in motion_profile(abs(target - start), self.rate, self.accel):
            self.send(start + (target - start) * f)
            self.sleep(TICK_S)

    def check(self, label):
        """Read the pulse back from the chip and compare it with the one asked for."""
        asked = angle_to_us(self.type, self.angle)
        got = self.pca.read_pulse_us(self.ch)
        ok = got is not None and abs(got - asked) <= READBACK_TOLERANCE_US
        self.mismatches += not ok
        print(f"    {label:9s} {self.angle:6.1f} deg  asked {asked:6.0f} us  chip {('off' if got is None else f'{got:6.0f} us'):>8s}  "
              f"{'ok' if ok else 'MISMATCH'}")

    def run(self, cycles):
        print(f"\n== {self.name} (channel {self.ch}, {self.type}): {self.lo:.0f} -> {self.hi:.0f} deg, "
              f"{self.rate:.0f} deg/s, {cycles} cycle(s)")
        print(f"  Starting from {self.start_angle():.1f} deg.")
        self.ramp(self.lo)
        self.sleep(self.dwell)
        self.check("low end")
        for cycle in range(1, cycles + 1):
            self.ramp(self.hi)
            self.sleep(self.dwell)
            self.check(f"high {cycle}/{cycles}")
            mid = (self.lo + self.hi) / 2
            self.ramp(self.lo)
            self.sleep(self.dwell)
            self.check(f"low {cycle}/{cycles}")
            self.ramp(mid)
            self.sleep(self.dwell / 2)
            self.check(f"mid {cycle}/{cycles}")
            self.ramp(self.lo)
        self.ramp(CENTER_DEG)
        self.sleep(self.dwell)
        self.check("centre")

    def seconds(self, cycles):
        """Time the sweep takes (without the time to reach the first angle)."""
        mid = (self.lo + self.hi) / 2
        t = move_time(self.hi - self.lo, self.rate, self.accel) + self.dwell
        t += move_time(self.hi - self.lo, self.rate, self.accel) + self.dwell
        t += move_time(mid - self.lo, self.rate, self.accel) + self.dwell / 2
        t += move_time(mid - self.lo, self.rate, self.accel)
        return cycles * t + move_time(abs(CENTER_DEG - self.lo), self.rate, self.accel) + 2 * self.dwell


def armbot_service_running():
    try:
        out = subprocess.run(["systemctl", "is-active", "armbot"], capture_output=True, text=True, timeout=5)
        return out.stdout.strip() == "active"
    except (OSError, subprocess.SubprocessError):
        return False


def main():
    ap = argparse.ArgumentParser(description="Sweep the base, shoulder and elbow servos from 30 to 150 degrees")
    ap.add_argument("--execute", action="store_true", help="drive the real servos (default: dry run)")
    ap.add_argument("--yes", action="store_true", help="skip the safety question")
    ap.add_argument("--joint", default="all", help="all, or a comma list of: " + ", ".join(JOINTS))
    ap.add_argument("--min", type=float, default=30.0, help="low end in degrees (default 30)")
    ap.add_argument("--max", type=float, default=150.0, help="high end in degrees (default 150)")
    ap.add_argument("--cycles", type=int, default=1, help="low-high-low sweeps per joint (default 1)")
    ap.add_argument("--speed", type=float, default=20.0, help="top speed in deg/s (default 20)")
    ap.add_argument("--accel", type=float, default=60.0, help="speeding up and slowing down, deg/s^2 (default 60)")
    ap.add_argument("--dwell", type=float, default=1.0, help="pause at each end in seconds (default 1)")
    ap.add_argument("--end", choices=("hold", "limp"), default="hold",
                    help="after the test: keep holding 90 degrees, or cut PWM on the tested joints")
    ap.add_argument("--config", default=str(CONFIG))
    args = ap.parse_args()

    if not 0 <= args.min < args.max <= 180:
        ap.error("--min and --max must satisfy 0 <= min < max <= 180")
    if args.speed <= 0 or args.accel <= 0 or args.cycles < 1 or args.dwell < 0:
        ap.error("--speed, --accel and --cycles must be positive and --dwell not negative")
    names = list(JOINTS) if args.joint == "all" else [n.strip() for n in args.joint.split(",")]
    unknown = [n for n in names if n not in JOINTS]
    if unknown:
        ap.error(f"unknown joint {', '.join(unknown)}; choose from {', '.join(JOINTS)}")

    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    by_name = {j["name"]: j for j in cfg["joints"]["list"]}
    hw = cfg["hardware"]

    if args.execute and armbot_service_running():
        raise SystemExit("The armbot service is running and also drives the arm. Stop it first:\n"
                         "  sudo systemctl stop armbot")
    if args.execute:
        pca = Pca9685(hw["i2c_bus"], int(hw["pca9685_address"], 0), keep_outputs=True)
    else:
        print("DRY RUN: no servo output. Add --execute to move the real arm.")
        pca = QuietDryRun()

    sweepers = [Sweeper(pca, by_name[n], args, not args.execute) for n in names]
    total = sum(s.seconds(args.cycles) for s in sweepers)
    print(f"\nPlan: {', '.join(s.name for s in sweepers)}, one at a time, {args.min:.0f}-{args.max:.0f} deg, "
          f"{args.cycles} cycle(s) each, about {total:.0f} s plus the move to the first angle.")

    if args.execute and not args.yes:
        print("The joints swing a long way: the arm can reach the table, its base or a cable.")
        if not input("Servo power on, area clear, hand on the arm? [y/n]: ").strip().lower().startswith("y"):
            print("Cancelled.")
            return
    try:
        for s in sweepers:
            s.run(args.cycles)
    except KeyboardInterrupt:
        print("\nStopped. The servos keep holding the last pose; hold the arm, then use --rest or servo_control_app.py.")
        pca.close()
        return
    bad = sum(s.mismatches for s in sweepers)
    print(f"\nDone. {'All pulses read back as asked.' if not bad else f'{bad} read-back MISMATCH(es): check power, wiring and the channel.'}")
    if args.end == "limp":
        for s in sweepers:
            pca.channel_off(s.ch)
        print("PWM cut on the tested joints: the arm is limp, hold it.")
    else:
        print(f"Holding {CENTER_DEG:.0f} deg on the tested joints.")
    pca.close()


if __name__ == "__main__":
    main()
