"""Six-slot servo tester for a PCA9685 on Raspberry Pi 5 (Tkinter GUI).

Each slot picks a PCA9685 channel (0-15) and a servo type, then nudges the
angle in 10 degree steps inside a 30-150 degree safety window. Moves are
ramped so the horn never jumps a full step at once.

Run:  python3 servo_control_app.py [--bus 1] [--addr 0x40] [--dry-run]

Safety behaviour:
- All outputs are switched off at start-up and on exit (window close / Ctrl+C).
- A slot sends nothing until its "Enable" box is ticked. Enabling drives the
  servo to the shown angle (90 by default); a hobby servo has no position
  feedback, so that first move cannot be ramped.
- Changing a slot's channel or type disables that slot (output off).
- Two slots can never share a channel.
- After a slot reaches its angle its PWM is cut (see HOLD_MS) so a stalled
  servo cannot overheat. A cut servo goes limp: a loaded joint may sag.
"""

import argparse
import signal
import time
import tkinter as tk
from tkinter import messagebox, ttk

NUM_SLOTS = 6
NUM_CHANNELS = 16
STEP_DEG = 10
MIN_DEG = 30
MAX_DEG = 150
HOME_DEG = 90
GRIP_DEG = 50
# Standby pose for slots 1-5 (base, shoulder, elbow, wrist roll, wrist pitch), shown angles in degrees.
STANDBY_DEG = [90, 85, 100, 90, 150]
UNGRIP_DEG = 150
# "Close & hold" keeps PWM on slots 1-5, the arm and wrist; the gripper relaxes as usual.
HOLD_SLOTS = 5
# Servo fitted to each slot: base, shoulder, elbow, wrist roll, wrist pitch, gripper.
SLOT_TYPES = ["MG996R", "TD-8125MG", "MG996R", "SG90", "SG90", "SG90"]
# A servo pushing against a load or end stop stalls and draws full current
# until it overheats. Cut a slot's PWM this long after it reaches its angle;
# the servo then goes limp and cools down. The open gripper always does this and
# the closed gripper never does; the other slots follow the "Cut PWM after move"
# checkbox.
HOLD_MS = 1500
TICK_MS = 20

# Pulse width at 0 and 180 degrees, plus ramp speed. The MG996R has more
# torque and heavier gears, so it ramps slower to spare worn gear trains.
SERVO_TYPES = {
    "SG90": {"us_0": 500.0, "us_180": 2400.0, "deg_per_s": 60.0},
    "MG996R": {"us_0": 500.0, "us_180": 2500.0, "deg_per_s": 45.0},
    # Futaba S3003: nylon gears, neutral ~1520 us, narrower travel than the MG996R.
    "S3003": {"us_0": 600.0, "us_180": 2400.0, "deg_per_s": 60.0},
    # TD-8125MG: 25 kg metal-gear digital servo, 180 degree version, 500-2500 us.
    "TD-8125MG": {"us_0": 500.0, "us_180": 2500.0, "deg_per_s": 45.0},
}

# PCA9685 registers
MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD
PRESCALE = 0xFE
SLEEP = 0x10
AI = 0x20
RESTART = 0x80
OUTDRV = 0x04
FULL_OFF = 0x10


class Pca9685:
    """Minimal PCA9685 driver over smbus (no extra pip packages needed)."""

    def __init__(self, bus=1, addr=0x40, freq_hz=50.0, keep_outputs=False):
        """keep_outputs: if the chip is already running at freq_hz, leave its PWM alone."""
        import smbus

        self.bus = smbus.SMBus(bus)
        self.addr = addr
        self.period_us = 1_000_000.0 / freq_hz
        prescale = max(3, min(255, round(25_000_000.0 / (4096.0 * freq_hz) - 1)))
        if keep_outputs:
            mode1 = self.bus.read_byte_data(addr, MODE1)
            if not mode1 & SLEEP and mode1 & AI and self.bus.read_byte_data(addr, PRESCALE) == prescale:
                return
        self.all_off()
        self.bus.write_byte_data(addr, MODE2, OUTDRV)
        self.bus.write_byte_data(addr, MODE1, SLEEP)
        self.bus.write_byte_data(addr, PRESCALE, prescale)
        self.bus.write_byte_data(addr, MODE1, AI)
        time.sleep(0.005)
        self.bus.write_byte_data(addr, MODE1, AI | RESTART)

    def set_pulse_us(self, ch, us):
        ticks = max(0, min(4095, round(us * 4096.0 / self.period_us)))
        self.bus.write_i2c_block_data(
            self.addr, LED0_ON_L + 4 * ch, [0, 0, ticks & 0xFF, ticks >> 8]
        )

    def read_pulse_us(self, ch):
        """Pulse the chip is sending on ch, or None if the channel is off."""
        low, high = (self.bus.read_byte_data(self.addr, LED0_ON_L + 4 * ch + i) for i in (2, 3))
        ticks = low | (high & 0x0F) << 8
        return None if high & FULL_OFF or not ticks else ticks * self.period_us / 4096.0

    def channel_off(self, ch):
        self.bus.write_byte_data(self.addr, LED0_ON_L + 4 * ch + 3, FULL_OFF)

    def all_off(self):
        self.bus.write_byte_data(self.addr, ALL_LED_OFF_H, FULL_OFF)

    def close(self):
        self.bus.close()


class DryRunPca9685:
    """Stand-in that prints instead of touching I2C."""

    def set_pulse_us(self, ch, us):
        print(f"[dry-run] ch{ch} -> {us:.0f} us")

    def read_pulse_us(self, ch):
        return None

    def channel_off(self, ch):
        print(f"[dry-run] ch{ch} off")

    def all_off(self):
        print("[dry-run] all off")

    def close(self):
        pass


def angle_to_us(servo_type, deg):
    t = SERVO_TYPES[servo_type]
    return t["us_0"] + (t["us_180"] - t["us_0"]) * deg / 180.0


def us_to_angle(servo_type, us):
    t = SERVO_TYPES[servo_type]
    return (us - t["us_0"]) * 180.0 / (t["us_180"] - t["us_0"])


class Slot:
    def __init__(self, app, parent, index, gripper=False):
        self.app = app
        self.index = index
        self.gripper = gripper
        self.relaxed = False  # gripper only: PWM cut after the hold time
        self.relax_job = None
        self.target = HOME_DEG  # what the user asked for
        self.current = None  # what was last sent; None = output off

        self.channel_var = tk.StringVar(value=str(index + 1))
        self.type_var = tk.StringVar(value=SLOT_TYPES[index])
        self.enabled_var = tk.BooleanVar(value=False)
        self.prev_channel = self.channel_var.get()
        self.prev_type = self.type_var.get()

        title = f"Slot {index + 1}" + (" – Gripper" if gripper else "")
        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.grid(row=index // 3, column=index % 3, padx=6, pady=6, sticky="nsew")

        ttk.Label(frame, text="Channel").grid(row=0, column=0, sticky="w")
        ch_box = ttk.Combobox(
            frame, textvariable=self.channel_var, width=5, state="readonly",
            values=[str(c) for c in range(NUM_CHANNELS)],
        )
        ch_box.grid(row=0, column=1, columnspan=2, sticky="ew", pady=2)
        ch_box.bind("<<ComboboxSelected>>", self.on_channel_change)

        ttk.Label(frame, text="Type").grid(row=1, column=0, sticky="w")
        type_box = ttk.Combobox(
            frame, textvariable=self.type_var, width=8, state="readonly",
            values=list(SERVO_TYPES),
        )
        type_box.grid(row=1, column=1, columnspan=2, sticky="ew", pady=2)
        type_box.bind("<<ComboboxSelected>>", self.on_type_change)

        ttk.Checkbutton(
            frame, text="Enable", variable=self.enabled_var, command=self.on_enable
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(4, 6))

        self.angle_label = ttk.Label(frame, width=7, anchor="center", font=("TkDefaultFont", 16, "bold"))
        self.grip_label = None
        if gripper:
            self.angle_label.grid(row=3, column=0, columnspan=3)
            ttk.Button(frame, text="Grip", command=lambda: self.go_to(GRIP_DEG)).grid(
                row=4, column=0, columnspan=2, sticky="ew", padx=(0, 4)
            )
            ttk.Button(frame, text="Ungrip", command=lambda: self.go_to(UNGRIP_DEG)).grid(
                row=4, column=2, sticky="ew"
            )
            self.grip_label = ttk.Label(frame, font=("TkDefaultFont", 11, "bold"))
            self.grip_label.grid(row=6, column=0, columnspan=3, pady=(6, 0))
        else:
            ttk.Button(frame, text="−", width=4, command=lambda: self.nudge(-STEP_DEG)).grid(
                row=3, column=0
            )
            self.angle_label.grid(row=3, column=1)
            ttk.Button(frame, text="+", width=4, command=lambda: self.nudge(STEP_DEG)).grid(
                row=3, column=2
            )

        ttk.Button(frame, text="Limp", command=self.go_limp).grid(
            row=5, column=0, columnspan=3, sticky="ew", pady=(4, 0)
        )

        self.status_label = ttk.Label(frame, foreground="gray")
        self.status_label.grid(row=7, column=0, columnspan=3, pady=(6, 0))
        self.refresh()

    @property
    def channel(self):
        return int(self.channel_var.get())

    def refresh(self):
        self.angle_label.config(text=f"{self.target}°")
        if self.current is None:
            self.status_label.config(text="output off", foreground="gray")
        elif self.relaxed:
            self.status_label.config(text="PWM off (limp, cooling)", foreground="#b26a00")
        else:
            us = angle_to_us(self.type_var.get(), self.current)
            moving = " (moving)" if abs(self.current - self.target) > 0.01 else ""
            self.status_label.config(text=f"{us:.0f} µs{moving}", foreground="green")
        if self.grip_label is not None:
            self.grip_label.config(text=self.grip_state())

    def grip_state(self):
        if self.current is None:
            return "Output off"
        moving = abs(self.current - self.target) > 0.01
        if self.target == GRIP_DEG:
            state = "Gripping…" if moving else "Gripped"
        elif self.target == UNGRIP_DEG:
            state = "Releasing…" if moving else "Open"
        else:
            state = f"Not set ({self.target}°)"
        return f"{state} (relaxed)" if self.relaxed else state

    def go_to(self, deg):
        self.target = max(MIN_DEG, min(MAX_DEG, deg))
        if self.gripper and self.current is not None:
            # A loaded gripper stalls on a slow ramp, so send the end position at once.
            self.current = float(self.target)
            if self.send():
                self.schedule_relax()
            else:
                self.current = None
                self.enabled_var.set(False)
        self.refresh()

    def schedule_relax(self):
        self.cancel_relax()
        self.relaxed = False
        self.relax_job = self.app.root.after(HOLD_MS, self.relax)

    def cancel_relax(self):
        if self.relax_job is not None:
            self.app.root.after_cancel(self.relax_job)
            self.relax_job = None

    def relax(self):
        self.relax_job = None
        if self.current is None:
            return
        # A closed gripper keeps its PWM so it holds the object until Ungrip or Limp.
        if not self.app.auto_cut_var.get() if not self.gripper else self.target == GRIP_DEG:
            return
        self.go_limp()

    def go_limp(self):
        """Cut this slot's PWM now. It stays enabled; the next move re-sends."""
        self.cancel_relax()
        if self.current is None:
            return
        # Stop a ramp in progress where it is.
        self.target = round(self.current)
        self.current = float(self.target)
        if self.app.pca_call(self.app.pca.channel_off, self.channel):
            self.relaxed = True
        self.refresh()

    def disable(self):
        self.cancel_relax()
        self.relaxed = False
        if self.current is not None:
            self.app.pca_call(self.app.pca.channel_off, self.channel)
        self.current = None
        self.enabled_var.set(False)
        self.refresh()

    def on_enable(self):
        if not self.enabled_var.get():
            self.disable()
            return
        # No position feedback: jump straight to the shown angle, then ramp from there.
        self.current = float(self.target)
        if not self.send():
            self.enabled_var.set(False)
            self.current = None
        else:
            self.schedule_relax()
        self.refresh()

    def on_channel_change(self, _event=None):
        new = self.channel_var.get()
        if new == self.prev_channel:
            return
        other = self.app.slot_using_channel(int(new), exclude=self)
        if other is not None:
            messagebox.showwarning(
                "Channel in use", f"Channel {new} is already used by Slot {other.index + 1}."
            )
            self.channel_var.set(self.prev_channel)
            return
        self.cancel_relax()
        self.relaxed = False
        if self.current is not None:
            self.app.pca_call(self.app.pca.channel_off, int(self.prev_channel))
            self.current = None
            self.enabled_var.set(False)
        self.prev_channel = new
        self.refresh()

    def on_type_change(self, _event=None):
        if self.type_var.get() != self.prev_type:
            self.prev_type = self.type_var.get()
            self.disable()

    def nudge(self, delta):
        self.target = max(MIN_DEG, min(MAX_DEG, self.target + delta))
        self.refresh()

    def tick(self, dt):
        if self.current is None or abs(self.current - self.target) < 0.01:
            return
        self.cancel_relax()
        self.relaxed = False
        step = SERVO_TYPES[self.type_var.get()]["deg_per_s"] * dt
        if abs(self.target - self.current) <= step:
            self.current = float(self.target)
        else:
            self.current += step if self.target > self.current else -step
        if not self.send():
            self.current = None
            self.enabled_var.set(False)
        elif self.current == self.target:
            self.schedule_relax()
        self.refresh()

    def send(self):
        us = angle_to_us(self.type_var.get(), self.current)
        return self.app.pca_call(self.app.pca.set_pulse_us, self.channel, us)


class App:
    def __init__(self, root, pca, mode_text):
        self.root = root
        self.pca = pca
        root.title("PCA9685 Servo Tester")

        top = ttk.Frame(root, padding=(10, 10, 10, 0))
        top.pack(fill="x")
        ttk.Label(top, text=mode_text).pack(side="left")
        tk.Button(
            top, text="ALL OFF", bg="#c62828", fg="white", activebackground="#8e0000",
            font=("TkDefaultFont", 11, "bold"), command=self.all_off,
        ).pack(side="right")
        ttk.Button(top, text="Close & hold", command=self.close_holding).pack(side="right", padx=(0, 10))
        ttk.Button(top, text="Standby", command=self.standby).pack(side="right", padx=(0, 4))
        self.auto_cut_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            top, text="Cut PWM after move", variable=self.auto_cut_var,
            command=self.on_auto_cut,
        ).pack(side="right", padx=10)

        grid = ttk.Frame(root, padding=6)
        grid.pack(fill="both", expand=True)
        self.slots = [Slot(self, grid, i, gripper=(i == NUM_SLOTS - 1)) for i in range(NUM_SLOTS)]

        ttk.Label(
            root, padding=(10, 0, 10, 10), foreground="gray",
            text=f"Step {STEP_DEG}°, limit {MIN_DEG}–{MAX_DEG}°. "
                 "Enabling a slot drives the servo to the shown angle. "
                 "Turn a slot off before touching its servo.",
        ).pack(fill="x")

        root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.last_tick = time.monotonic()
        root.after(TICK_MS, self.tick)

    def slot_using_channel(self, ch, exclude=None):
        for s in self.slots:
            if s is not exclude and s.channel == ch:
                return s
        return None

    def pca_call(self, fn, *args):
        try:
            fn(*args)
            return True
        except OSError as e:
            messagebox.showerror("I2C error", f"{e}\nCheck wiring and power, then re-enable the slot.")
            return False

    def standby(self):
        # Enabled slots ramp there; slots that are off jump there when enabled.
        # The arm has to stay up in this pose, so stop cutting PWM after moves.
        self.auto_cut_var.set(False)
        for slot, deg in zip(self.slots, STANDBY_DEG):
            # Another program may have moved the servo; ramp from where the chip really is.
            try:
                us = self.pca.read_pulse_us(slot.channel)
            except OSError:
                us = None
            if us is not None:
                slot.cancel_relax()
                slot.current = us_to_angle(slot.type_var.get(), us)
                slot.relaxed = False
                slot.enabled_var.set(True)
            if slot.current is None:
                slot.target = deg
                slot.enabled_var.set(True)
                slot.on_enable()
            else:
                slot.go_to(deg)
                if slot.relaxed and slot.current == slot.target:
                    slot.relaxed = not slot.send()  # already there, so no ramp will re-send
                    slot.refresh()

    def close_holding(self):
        """Exit but leave slots 1-3 powered, ready for pick_and_place.py."""
        held = self.slots[:HOLD_SLOTS]
        if any(s.current is None or abs(s.current - s.target) > 0.01 for s in held):
            messagebox.showwarning(
                "Not holding", f"Slots 1-{HOLD_SLOTS} must be on and finished moving. Press Standby first."
            )
            return
        for s in self.slots:
            s.cancel_relax()
        if not all(s.send() for s in held):
            return
        for s in self.slots[HOLD_SLOTS:]:
            if s.current is not None:
                self.pca_call(self.pca.channel_off, s.channel)
        self.pca.close()
        self.root.destroy()

    def all_off(self):
        self.pca_call(self.pca.all_off)
        for s in self.slots:
            s.cancel_relax()
            s.relaxed = False
            s.current = None
            s.enabled_var.set(False)
            s.refresh()

    def on_auto_cut(self):
        # Turning it on: relax every slot already resting at its angle.
        if self.auto_cut_var.get():
            for s in self.slots:
                if s.current == s.target and not s.relaxed and s.relax_job is None:
                    s.schedule_relax()

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        for s in self.slots:
            s.tick(dt)
        self.root.after(TICK_MS, self.tick)

    def shutdown(self):
        try:
            self.pca.all_off()
        except OSError:
            pass
        self.pca.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default 1)")
    parser.add_argument("--addr", type=lambda s: int(s, 0), default=0x40, help="PCA9685 address")
    parser.add_argument("--dry-run", action="store_true", help="print pulses, no I2C")
    args = parser.parse_args()

    if args.dry_run:
        pca, mode = DryRunPca9685(), "DRY RUN: no I2C output"
    else:
        try:
            pca = Pca9685(args.bus, args.addr)
        except OSError as e:
            raise SystemExit(
                f"Cannot reach PCA9685 on /dev/i2c-{args.bus} at 0x{args.addr:02x}: {e}\n"
                "Check wiring and `i2cdetect -y 1`, or run with --dry-run."
            )
        mode = f"PCA9685 on /dev/i2c-{args.bus} at 0x{args.addr:02x}, 50 Hz"

    root = tk.Tk()
    app = App(root, pca, mode)
    # Ctrl+C in the terminal: Tk only sees signals between events, so poll.
    signal.signal(signal.SIGINT, lambda *_: app.shutdown())

    def keep_signals_alive():
        root.after(200, keep_signals_alive)

    keep_signals_alive()
    root.mainloop()


if __name__ == "__main__":
    main()
