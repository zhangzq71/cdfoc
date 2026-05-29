#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2026, DUKELEC, Inc.

"""CDFOC motor control tool

Drives the motor with sinusoidal trajectory or constant-speed rotation.

Features (sine):
  - Position mode sine wave: position = amplitude * sin(2π * freq * t)
  - Speed mode sine wave: speed = amplitude * sin(2π * freq * t)

Features (speed/tp constant rotation):
  - speed mode: direct speed PID control (state=3), more stable RPM
  - tp mode: trap planner position tracking (state=5), accel/decel control
  - Target speed in RPM, configurable direction
  - Real-time RPM feedback

Args:
  --dev DEV             # serial port or match string, default: ttyACM0
  --baud BAUD           # default: 115200
  --local-mac MAC       # default: 0x00
  --target-addr ADDR    # default: 00:00:fe
  --cfg FILE            # register config JSON (same as cdbus_gui/configs/cdfoc-v7.json)
  --mode MODE           # 'pos', 'speed' (sine), or 'tp' (constant RPM), default: pos
  --amplitude VAL       # sine: amplitude (encoder counts for pos, speed for speed)
  --freq VAL            # sine: frequency in Hz, default: 0.5
  --duration SEC        # run duration in seconds, default: 10.0 (0 = run forever)
  --rate HZ             # sine: target update rate in Hz, default: 100
  --kp VAL              # pid_speed_kp (speed) or pid_pos.kp (pos/tp)
  --ki VAL              # pid_speed_ki (speed) or pid_pos.ki (pos/tp)
  --rpm RPM             # speed/tp mode: target speed in RPM (float)
  --dir {cw,ccw}        # speed/tp mode: rotation direction, default: cw
  --tp-speed VAL        # tp mode: override tp_speed (raw units)
  --tp-accel VAL        # tp mode: override tp_accel (raw units)
  --keep-running        # do not stop motor on exit
"""

import os
import sys
import math
import time
import struct
import threading
from argparse import ArgumentParser

sys.path.append(os.path.join(os.path.dirname(__file__), "./pycdnet"))

from cdnet.dev.cdbus_serial import CDBusSerial
from cdnet.dispatch import CDNetIntf, CDNetSocket


class RegDb:
    def __init__(self, cfg_path: str):
        import json5
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json5.load(f)
        self.regs = {}
        for item in cfg["reg"]["list"]:
            addr, length, fmt, _show, name = item[:5]
            self.regs[name] = {"addr": int(addr), "len": int(length), "fmt": fmt}

    def get(self, name: str):
        if name not in self.regs:
            raise KeyError(f"register not found: {name}")
        return self.regs[name]


class FocSineController:
    def __init__(self, dev: str, baud: int, local_mac: int, target_addr: str, reg_db: RegDb):
        self.target_addr = target_addr
        self.reg_db = reg_db
        self.dev = CDBusSerial(dev, baud=baud)
        CDNetIntf(self.dev, mac=local_mac)
        self.sock = CDNetSocket(("", 0x40))
        # Drain debug packets to avoid "port 9 not found" spam
        self.sock_dbg = CDNetSocket(("", 0x9))
        self.lock = threading.Lock()
        self._dbg_stop = threading.Event()
        self._dbg_thread = threading.Thread(target=self._drain_dbg, daemon=True)
        self._dbg_thread.start()

    def _drain_dbg(self):
        while not self._dbg_stop.is_set():
            self.sock_dbg.recvfrom(timeout=0.8)

    def _pack_value(self, fmt: str, value):
        fmts = {"f": "<f", "i": "<i", "I": "<I", "b": "<b", "B": "<B", "h": "<h", "H": "<H"}
        if fmt not in fmts:
            raise ValueError(f"unsupported fmt: {fmt}")
        return struct.pack(fmts[fmt], int(value) if fmt != "f" else float(value))

    def _unpack_value(self, fmt: str, data: bytes):
        fmts = {"f": "<f", "i": "<i", "I": "<I", "b": "<b", "B": "<B", "h": "<h", "H": "<H"}
        if fmt not in fmts:
            raise ValueError(f"unsupported fmt: {fmt}")
        return struct.unpack(fmts[fmt], data)[0]

    def write_reg(self, name: str, value, retry: int = 3):
        reg = self.reg_db.get(name)
        payload = b"\x20" + struct.pack("<H", reg["addr"]) + self._pack_value(reg["fmt"], value)
        for _ in range(retry):
            with self.lock:
                self.sock.clear()
                self.sock.sendto(payload, (self.target_addr, 0x5))
                dat, src = self.sock.recvfrom(timeout=0.8)
            if src and src[0] == self.target_addr and src[1] == 0x5 and dat and dat[0] == 0:
                return
        raise RuntimeError(f"write failed: {name}={value}")

    def read_reg(self, name: str, retry: int = 3):
        reg = self.reg_db.get(name)
        payload = b"\x00" + struct.pack("<HB", reg["addr"], reg["len"])
        for _ in range(retry):
            with self.lock:
                self.sock.clear()
                self.sock.sendto(payload, (self.target_addr, 0x5))
                dat, src = self.sock.recvfrom(timeout=0.8)
            if src and src[0] == self.target_addr and src[1] == 0x5 and dat and dat[0] == 0:
                return self._unpack_value(reg["fmt"], dat[1:1 + reg["len"]])
        raise RuntimeError(f"read failed: {name}")

    def close(self):
        self._dbg_stop.set()
        self._dbg_thread.join(timeout=1.0)


def build_parser():
    parser = ArgumentParser(description="CDFOC sine wave motion control tool")
    parser.add_argument("--dev", default="ttyACM0")
    parser.add_argument("--baud", default="30000000")
    parser.add_argument("--local-mac", default="0x00")
    parser.add_argument("--target-addr", default="00:00:fe")
    parser.add_argument(
        "--cfg",
        default=os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "..", "cdbus_gui", "configs", "cdfoc-v7.json")
        ),
    )
    parser.add_argument("--mode", choices=["pos", "speed", "tp"], default="pos",
                        help="'pos': sine position; 'speed': sine/const-speed; 'tp': const-RPM (state=5)")
    parser.add_argument("--amplitude", type=float, default=10000.0,
                        help="sine: amplitude (encoder counts for pos, speed value for speed)")
    parser.add_argument("--freq", type=float, default=0.5,
                        help="sine: frequency in Hz")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="run duration in seconds (0 = run forever)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="sine: target update rate in Hz")
    parser.add_argument("--kp", type=float, default=None,
                        help="pid_speed_kp (speed) or pid_pos.kp (pos/tp)")
    parser.add_argument("--ki", type=float, default=None,
                        help="pid_speed_ki (speed) or pid_pos.ki (pos/tp)")
    parser.add_argument("--rpm", type=float, default=None,
                        help="speed/tp: target speed in RPM (float)")
    parser.add_argument("--dir", choices=["cw", "ccw"], default="cw",
                        help="speed/tp: rotation direction, default: cw")
    parser.add_argument("--tp-speed", type=int, default=None,
                        help="tp mode: override tp_speed (raw units)")
    parser.add_argument("--tp-accel", type=int, default=None,
                        help="tp mode: override tp_accel (raw units)")
    parser.add_argument("--keep-running", action="store_true",
                        help="do not stop motor on exit")
    return parser


def main():
    args = build_parser().parse_args()

    baud = int(args.baud, 0)
    local_mac = int(args.local_mac, 0)

    reg_db = RegDb(args.cfg)
    ctrl = FocSineController(
        dev=args.dev,
        baud=baud,
        local_mac=local_mac,
        target_addr=args.target_addr,
        reg_db=reg_db,
    )

    # --- Constants ---
    ENC_CPR = 65536  # 16-bit encoder counts per revolution

    # --- Determine if constant-rpm mode ---
    is_const_rpm = (args.mode in ("speed", "tp")) and args.rpm is not None
    if args.mode == "tp" and args.rpm is None:
        print("error: --rpm is required in tp mode")
        sys.exit(1)

    # --- Print startup info ---
    if is_const_rpm:
        print(f"connected: dev={args.dev}, baud={baud}, target={args.target_addr}")
        print(f"mode={args.mode} (constant rpm), rpm={args.rpm}, dir={args.dir}, "
              f"duration={args.duration}s")
    else:
        print(f"connected: dev={args.dev}, baud={baud}, target={args.target_addr}")
        print(f"mode={args.mode}, amplitude={args.amplitude}, freq={args.freq} Hz, "
              f"update_rate={args.rate} Hz")

    # --- Determine mode-specific settings ---
    if args.mode in ("pos", "tp"):
        kp_regs = ["pid_kp", "pid_pos_kp"]
        ki_regs = ["pid_ki", "pid_pos_ki"]
        state_target = 5 if args.mode == "tp" else 4
    else:  # speed
        kp_regs = ["pid_speed_kp"]
        ki_regs = ["pid_speed_ki"]
        state_target = 3

    # --- Attempt to set PID gains ---
    if args.kp is not None:
        for name in kp_regs:
            try:
                ctrl.write_reg(name, args.kp)
                print(f"{name} <- {args.kp}")
                break
            except KeyError:
                continue

    if args.ki is not None:
        for name in ki_regs:
            try:
                ctrl.write_reg(name, args.ki)
                print(f"{name} <- {args.ki}")
                break
            except KeyError:
                continue

    # --- Constant-rpm: convert RPM → encoder counts / sec ---
    rpm_cps = 0.0  # encoder counts per second
    if is_const_rpm:
        rpm_cps = abs(args.rpm) * ENC_CPR / 60.0

    # --- Configure trap planner (tp mode) ---
    if args.mode == "tp":
        speed_raw = int(rpm_cps)
        accel_raw = int(speed_raw * 5)

        tp_speed_val = args.tp_speed if args.tp_speed is not None else speed_raw
        tp_accel_val = args.tp_accel if args.tp_accel is not None else accel_raw

        tp_speed_val = max(1, min(tp_speed_val, 0xFFFFFFFF))
        tp_accel_val = max(1, min(tp_accel_val, 0xFFFFFFFF))

        ctrl.write_reg("tp_speed", tp_speed_val)
        ctrl.write_reg("tp_accel", tp_accel_val)
        print(f"tp_speed <- {tp_speed_val} ({args.rpm} rpm → {speed_raw} cnt/s)")
        print(f"tp_accel <- {tp_accel_val}")

    # Read the current position as a baseline offset
    pos_offset = 0
    try:
        pos_offset = ctrl.read_reg("cal_pos")
        print(f"current cal_pos offset: {pos_offset}")
    except Exception:
        pass

    # Stop motor first, then switch to target mode
    ctrl.write_reg("state", 0)
    time.sleep(0.5)

    ctrl.write_reg("state", state_target)
    time.sleep(0.2)

    print(f"motor started, state={state_target} ({args.mode} mode)")

    # --- Constant-RPM main loop (speed or tp mode) ---
    if is_const_rpm:
        ctrl_period = 0.05  # 20 Hz
        dir_sign = 1 if args.dir == "cw" else -1
        target_speed = rpm_cps * dir_sign  # signed cnt/s
        t_start = time.time()

        if args.mode == "speed":
            # Speed mode: write cal_speed directly (once, firmware holds it)
            ctrl.write_reg("cal_speed", target_speed, retry=3)

        try:
            while True:
                loop_start = time.time()
                t = time.time() - t_start

                if args.duration > 0 and t >= args.duration:
                    print(f"\nduration {args.duration}s reached")
                    break

                if args.mode == "tp":
                    # Extend tp_pos ahead of current position
                    cal_pos = ctrl.read_reg("cal_pos", retry=1)
                    extend = int(ENC_CPR * 10 * dir_sign)
                    target_pos = cal_pos + extend
                    ctrl.write_reg("tp_pos", target_pos, retry=1)

                # Read back speed / rpm
                try:
                    sen_speed = ctrl.read_reg("sen_speed", retry=1)
                    sen_rpm = ctrl.read_reg("sen_rpm_avg", retry=1)
                    print(f"t={t:.3f}\ttgt_rpm={args.rpm:>8.1f}\t"
                          f"speed={sen_speed:>8.1f}\trpm={sen_rpm:>8.1f}")
                except Exception:
                    pass

                elapsed = time.time() - loop_start
                sleep_time = ctrl_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nkeyboard interrupted")

        finally:
            if not args.keep_running:
                try:
                    if args.mode == "speed":
                        ctrl.write_reg("cal_speed", 0.0)
                    else:
                        ctrl.write_reg("tp_pos", cal_pos if 'cal_pos' in dir() else 0)
                    ctrl.write_reg("state", 0)
                    print("exit cleanup: motor stopped")
                except Exception as e:
                    print(f"exit cleanup warning: {e}")
            ctrl.close()
        return

    # --- Sine wave loop (pos / speed modes) ---
    period = 1.0 / args.rate
    t_start = time.time()

    try:
        while True:
            loop_start = time.time()
            t = time.time() - t_start

            if args.duration > 0 and t >= args.duration:
                print(f"\nduration {args.duration}s reached")
                break

            # Calculate sine value
            if args.mode == "speed":
                target = args.amplitude * math.sin(2 * math.pi * args.freq * t)
                ctrl.write_reg("cal_speed", target, retry=1)
            else:  # pos
                target = int(args.amplitude * math.sin(2 * math.pi * args.freq * t)) + pos_offset
                ctrl.write_reg("cal_pos", target, retry=1)

            # Read back actual position for feedback
            try:
                cal_pos = ctrl.read_reg("cal_pos", retry=1)
                sen_pos = ctrl.read_reg("sen_pos", retry=1)
                print(f"t={t:.3f}\ttarget={target:>10}\tcal_pos={cal_pos:>10}\tsen_pos={sen_pos:>10}")
            except Exception:
                pass

            # Sleep to maintain the target update rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nkeyboard interrupted")

    finally:
        if not args.keep_running:
            try:
                if args.mode == "speed":
                    ctrl.write_reg("cal_speed", 0.0)
                else:
                    ctrl.write_reg("cal_pos", pos_offset)
                ctrl.write_reg("state", 0)
                print("exit cleanup: motor stopped")
            except Exception as e:
                print(f"exit cleanup warning: {e}")
        ctrl.close()


if __name__ == "__main__":
    main()
