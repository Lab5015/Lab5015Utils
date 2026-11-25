#!/usr/bin/env python3
"""
Control of a Thorlabs HDR50 rotation stage driven by a Thorlabs UMC1
(Universal Motor Controller) over USB.

Usage examples:
    python3 thorlab_rotating.py                # interactive mode
    python3 thorlab_rotating.py --pos          # print current angle
    python3 thorlab_rotating.py --angle 45     # absolute move to 45 deg
    python3 thorlab_rotating.py --rel -10      # relative move by -10 deg
    python3 thorlab_rotating.py --home         # home on the switch (absolute angle)
    python3 thorlab_rotating.py --zero         # current position becomes 0 deg

It can also be imported:
    from thorlab_rotating import RotatingStage
    with RotatingStage() as rs:
        rs.move_to(30)
"""
import argparse
import json
import os
import time
import warnings

warnings.filterwarnings("ignore")

from pylablib.devices import Thorlabs
from pylablib.devices.Thorlabs import kinesis

DEVICE = "/dev/serial/by-id/usb-Thorlabs_Universal_Motor_Controller_121000613-if00-port0"

# HDR50: 200 full steps/rev, 66:1 worm gear, 2048 microsteps/step
# -> 409600 * 66 / 360 ~= 75091 counts per degree.
# If the measured angle does not match, calibrate this value.
SCALE = 75091  # counts per degree

# Angle of the home switch in the user's reference frame (measured on
# 2026-11-23: switch found at -182.615 deg from the 0 of the stage scale).
# After home() the stage is at this angle, so 0 deg = 0 of the stage scale.
HOME_ANGLE = -182.615

# The UMC1 ignores the "set position counter" commands, so the angle zero is
# kept in software: angle = counter/SCALE - offset. The offset is saved here.
# NOTE: the controller counter restarts from 0 at power-up -> after switching
# the UMC1 off/on, run --home (or set the zero again).
OFFSET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rotating_offset.json")


def _cycle_rts_keep_high(instr):
    # The UMC1 stops answering if RTS is left low, which is what pylablib does
    # by default on connection: keep it high instead.
    instr.instr.setRTS(1)
    time.sleep(0.05)
    instr.instr.flushInput()
    instr.instr.flushOutput()
    time.sleep(0.05)


kinesis.BasicKinesisDevice._cycle_rts = staticmethod(_cycle_rts_keep_high)


class RotatingStage:
    def __init__(self, device=DEVICE, scale=SCALE):
        self.scale = scale
        # UMC1 behaves like a rack system (controller 0x11 + channel 0x21)
        self.stage = Thorlabs.KinesisMotor(("serial", device), is_rack_system=True)
        self.offset = self._load_offset()
        self.enable()

    # ---- software zero ----
    @staticmethod
    def _load_offset():
        try:
            with open(OFFSET_FILE) as f:
                return float(json.load(f)["offset_deg"])
        except (OSError, ValueError, KeyError):
            return 0.0

    def _save_offset(self, offset):
        self.offset = offset
        with open(OFFSET_FILE, "w") as f:
            json.dump({"offset_deg": offset, "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, f)

    def _raw_angle(self):
        """Angle from the controller counter, without offset."""
        return self.stage.get_position() / self.scale

    def enable(self):
        """Enable the motor channel (the controller ignores moves otherwise)."""
        st = self.stage.get_status()
        if "enabled" not in st:
            self.stage.send_comm(0x0210, 1, 0x01, dest=("channel", 1))
            time.sleep(0.5)
            st = self.stage.get_status()
        if "enabled" not in st:
            print(f"WARNING: motor channel NOT enabled, moves will be ignored. Status: {st}")
            if "interlock" in st:
                print("         'interlock' flag is set: check cables / interlock connector.")
        return "enabled" in st

    # ---- context manager ----
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.stage.close()

    # ---- basic functions ----
    def get_angle(self):
        """Current angle in degrees."""
        return self._raw_angle() - self.offset

    def _wait(self, target, timeout):
        """Wait for the move to end; stop the motor if it takes too long or
        if it runs away from the target (safety)."""
        time.sleep(0.3)  # let the controller flag the move as active
        start = self.get_angle()
        t0 = time.time()
        try:
            while "active" in self.stage.get_status():
                angle = self.get_angle()
                runaway = abs(angle - target) > abs(start - target) + 5
                if time.time() - t0 > timeout or runaway:
                    self.stop()
                    print(f"WARNING: move stopped at {self.get_angle():.3f} deg "
                          f"({'timeout' if not runaway else 'moving away from target'})")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
            raise
        return self.get_angle()

    def _timeout(self, delta):
        # ~1.5 deg/s at default speed: allow 1 s/deg + margin
        return 20 + abs(delta)

    def move_to(self, angle, wait=True, timeout=None):
        """Absolute move to `angle` degrees."""
        if timeout is None:
            timeout = self._timeout(angle - self.get_angle())
        self.stage.move_to(int(round((angle + self.offset) * self.scale)))
        return self._wait(angle, timeout) if wait else self.get_angle()

    def move_by(self, delta, wait=True, timeout=None):
        """Relative move by `delta` degrees."""
        if timeout is None:
            timeout = self._timeout(delta)
        target = self.get_angle() + delta
        self.stage.move_by(int(round(delta * self.scale)))
        return self._wait(target, timeout) if wait else self.get_angle()

    def go_zero(self):
        """Go back to 0 deg."""
        return self.move_to(0)

    def home(self, timeout=330):
        """Hardware homing on the HDR50 home switch.

        The controller's own homing command does not work with the UMC1, so it
        is done here: rotate backwards until the home switch (CCW limit input,
        'make' polarity) stops the motor, then set the angle to HOME_ANGLE.
        Takes up to ~4 min (one full turn).
        """
        s = self.stage
        s.setup_limit_switch(hw_kind_cw="ignore", hw_kind_ccw="make")
        time.sleep(0.3)
        # if already on the switch, move away first so the approach is
        # always from the same side
        if "hw_bk_lim" in s.get_status():
            s.move_by(int(10 * self.scale))
            self._wait(self.get_angle() + 10, 30)
            if "hw_bk_lim" in s.get_status():
                raise RuntimeError("homing failed: could not move away from the home switch")
        s.move_by(int(-370 * self.scale))
        time.sleep(0.3)
        t0 = time.time()
        try:
            while "active" in s.get_status():
                if time.time() - t0 > timeout:
                    self.stop()
                    raise RuntimeError("homing timeout: home switch not found")
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.stop()
            raise
        if "hw_bk_lim" not in s.get_status():
            raise RuntimeError("homing failed: motor stopped but home switch not active")
        self._save_offset(self._raw_angle() - HOME_ANGLE)
        return self.get_angle()

    def is_enabled(self):
        return "enabled" in self.stage.get_status()

    def set_zero(self):
        """Define the current position as 0 deg (without moving)."""
        self._save_offset(self._raw_angle())

    def stop(self):
        # immediate stop, without waiting (pylablib's stop() may hang on UMC1)
        self.stage.send_comm(0x0465, 1, 0x01, dest=("channel", 1))
        time.sleep(0.3)

    def status(self):
        return self.stage.get_status()


def interactive(rs):
    print("Commands:  <deg> = absolute move | r <deg> = relative move | "
          "h = home (switch) | 0 = go to 0 deg | z = set current as 0 | p = position | q = quit")
    while True:
        try:
            cmd = input(f"[{rs.get_angle():.3f} deg] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not cmd:
            continue
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("p", "pos"):
                print(f"Angle: {rs.get_angle():.3f} deg")
            elif cmd in ("h", "home"):
                print("Homing (up to ~4 min)...")
                print(f"Homed: {rs.home():.3f} deg")
            elif cmd in ("z", "zero"):
                rs.set_zero()
                print("Current position set to 0 deg")
            elif cmd.startswith("r"):
                print(f"Moved to {rs.move_by(float(cmd[1:])):.3f} deg")
            else:
                print(f"Moved to {rs.move_to(float(cmd)):.3f} deg")
        except ValueError:
            print("Invalid command")
        except RuntimeError as e:
            print(f"ERROR: {e}")
        except KeyboardInterrupt:
            rs.stop()
            print("\nStopped")


def main():
    parser = argparse.ArgumentParser(description="Thorlabs HDR50 rotation stage (UMC1 controller)")
    g = parser.add_mutually_exclusive_group()
    g.add_argument("--angle", type=float, help="absolute move to ANGLE degrees")
    g.add_argument("--rel", type=float, help="relative move by REL degrees")
    g.add_argument("--home", action="store_true", help="home on the stage switch (sets the absolute angle)")
    g.add_argument("--zero", action="store_true", help="set current position as 0 deg")
    g.add_argument("--pos", action="store_true", help="print current angle and exit")
    parser.add_argument("--device", default=DEVICE, help="serial device")
    parser.add_argument("--scale", type=float, default=SCALE, help="counts per degree")
    args = parser.parse_args()

    with RotatingStage(args.device, args.scale) as rs:
        print(f"Connected to {rs.stage.get_device_info().model_no}, angle = {rs.get_angle():.3f} deg, "
              f"motor {'ENABLED' if rs.is_enabled() else 'DISABLED'}")
        if args.pos:
            pass
        elif args.angle is not None:
            print(f"Moved to {rs.move_to(args.angle):.3f} deg")
        elif args.rel is not None:
            print(f"Moved to {rs.move_by(args.rel):.3f} deg")
        elif args.home:
            print("Homing (up to ~4 min)...")
            print(f"Homed: {rs.home():.3f} deg")
        elif args.zero:
            rs.set_zero()
            print("Current position set to 0 deg")
        else:
            interactive(rs)


if __name__ == "__main__":
    main()
