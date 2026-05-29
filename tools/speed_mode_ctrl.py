#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2026, DUKELEC, Inc.

"""CDFOC speed mode control tool

Features:
  - Enter speed mode (state = 3)
  - Tune pid_speed_kp / pid_speed_ki / cal_speed at runtime
  - Read back key runtime values

Args:
  --dev DEV             # serial port or match string, default: ttyACM0
  --baud BAUD           # default: 115200
  --local-mac MAC       # default: 0x00
  --target-addr ADDR    # default: 00:00:fe
  --cfg FILE            # default: ../../cdbus_gui/configs/cdfoc-v7.json
  --kp VAL              # initial pid_speed_kp, default: keep device current value
  --ki VAL              # initial pid_speed_ki, default: keep device current value
  --speed VAL           # initial cal_speed, default: 0
    --pos-sec SEC          # position print period, default: 0.5
    --no-pos-stream        # disable continuous position print
  --keep-running        # do not stop motor on exit

Runtime commands:
  kp <float>            # update pid_speed_kp
  ki <float>            # update pid_speed_ki
  speed <float>         # update cal_speed
  state <0|2|3|4|5>     # update state
  show                  # print key values
  save                  # write save_conf = 1
  stop                  # set cal_speed=0, state=0
  start                 # set state=3
  help                  # show command help
  quit / exit           # leave tool
"""

import os
import sys
import json5
import struct
import threading
from argparse import ArgumentParser

sys.path.append(os.path.join(os.path.dirname(__file__), "./pycdnet"))

from cdnet.dev.cdbus_serial import CDBusSerial
from cdnet.dispatch import CDNetIntf, CDNetSocket


class RegDb:
    def __init__(self, cfg_path: str):
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


class FocSpeedController:
    def __init__(self, dev: str, baud: int, local_mac: int, target_addr: str, reg_db: RegDb):
        self.target_addr = target_addr
        self.reg_db = reg_db
        self.dev = CDBusSerial(dev, baud=baud)
        CDNetIntf(self.dev, mac=local_mac)
        self.sock = CDNetSocket(("", 0x40))
        # Bind debug port to avoid "port 9 not found, drop" spam from firmware debug packets.
        self.sock_dbg = CDNetSocket(("", 0x9))
        self.lock = threading.Lock()
        self._dbg_stop_event = threading.Event()
        self._dbg_drain_thread = threading.Thread(target=self._drain_dbg, daemon=True)
        self._dbg_drain_thread.start()

    def _drain_dbg(self):
        while not self._dbg_stop_event.is_set():
            self.sock_dbg.recvfrom(timeout=0.8)

    def _pack_value(self, fmt: str, value):
        if fmt == "f":
            return struct.pack("<f", float(value))
        if fmt == "i":
            return struct.pack("<i", int(value))
        if fmt == "I":
            return struct.pack("<I", int(value))
        if fmt == "b":
            return struct.pack("<b", int(value))
        if fmt == "B":
            return struct.pack("<B", int(value))
        if fmt == "h":
            return struct.pack("<h", int(value))
        if fmt == "H":
            return struct.pack("<H", int(value))
        raise ValueError(f"unsupported fmt: {fmt}")

    def _unpack_value(self, fmt: str, data: bytes):
        if fmt == "f":
            return struct.unpack("<f", data)[0]
        if fmt == "i":
            return struct.unpack("<i", data)[0]
        if fmt == "I":
            return struct.unpack("<I", data)[0]
        if fmt == "b":
            return struct.unpack("<b", data)[0]
        if fmt == "B":
            return struct.unpack("<B", data)[0]
        if fmt == "h":
            return struct.unpack("<h", data)[0]
        if fmt == "H":
            return struct.unpack("<H", data)[0]
        raise ValueError(f"unsupported fmt: {fmt}")

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


def build_parser():
    parser = ArgumentParser(description="CDFOC speed mode control tool")
    parser.add_argument("--dev", default="ttyACM0")
    parser.add_argument("--baud", default="115200")
    parser.add_argument("--local-mac", default="0x00")
    parser.add_argument("--target-addr", default="00:00:fe")
    parser.add_argument(
        "--cfg",
        default=os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "..", "cdbus_gui", "configs", "cdfoc-v7.json")
        ),
    )
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--ki", type=float, default=None)
    parser.add_argument("--speed", type=float, default=0.0)
    parser.add_argument("--pos-sec", type=float, default=0.5)
    parser.add_argument("--no-pos-stream", action="store_true")
    parser.add_argument("--keep-running", action="store_true")
    return parser


def print_runtime_help():
    print("commands: kp <v> | ki <v> | speed <v> | state <v> | pos on|off | show | save | stop | start | help | exit")


def run_pos_stream(
    ctrl: FocSpeedController,
    interval_sec: float,
    stop_event: threading.Event,
    pos_stream_event: threading.Event,
):
    while not stop_event.is_set():
        if not pos_stream_event.is_set():
            stop_event.wait(interval_sec)
            continue
        try:
            cal_pos = ctrl.read_reg("cal_pos")
            sen_pos = ctrl.read_reg("sen_pos")
            print(f"cal_pos={cal_pos} sen_pos={sen_pos}")
        except Exception as e:
            print(f"position read warning: {e}")
        stop_event.wait(interval_sec)


def main():
    args = build_parser().parse_args()

    baud = int(args.baud, 0)
    local_mac = int(args.local_mac, 0)

    reg_db = RegDb(args.cfg)
    ctrl = FocSpeedController(
        dev=args.dev,
        baud=baud,
        local_mac=local_mac,
        target_addr=args.target_addr,
        reg_db=reg_db,
    )

    print(f"connected: dev={args.dev}, baud={baud}, target={args.target_addr}")

    # Initial setup for speed mode.
    if args.kp is not None:
        ctrl.write_reg("pid_speed_kp", args.kp)
    if args.ki is not None:
        ctrl.write_reg("pid_speed_ki", args.ki)
    ctrl.write_reg("state", 3)
    ctrl.write_reg("cal_speed", args.speed)

    print("initial setup done")
    try:
        print(
            "current: "
            f"pid_speed_kp={ctrl.read_reg('pid_speed_kp'):.6f}, "
            f"pid_speed_ki={ctrl.read_reg('pid_speed_ki'):.6f}, "
            f"cal_speed={ctrl.read_reg('cal_speed'):.3f}"
        )
    except Exception as e:
        print(f"readback warning: {e}")

    stop_event = threading.Event()
    pos_stream_event = threading.Event()
    if not args.no_pos_stream:
        pos_stream_event.set()
    t = threading.Thread(
        target=run_pos_stream,
        args=(ctrl, args.pos_sec, stop_event, pos_stream_event),
        daemon=True,
    )
    t.start()

    print_runtime_help()

    try:
        while True:
            line = input("speed-ctrl> ").strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit"):
                break
            if cmd == "help":
                print_runtime_help()
                continue
            if cmd == "pos" and len(parts) == 2:
                sw = parts[1].lower()
                if sw == "on":
                    pos_stream_event.set()
                    print("position output enabled")
                elif sw == "off":
                    pos_stream_event.clear()
                    print("position output disabled")
                else:
                    print("usage: pos on|off")
                continue
            if cmd == "kp" and len(parts) == 2:
                val = float(parts[1])
                ctrl.write_reg("pid_speed_kp", val)
                print(f"pid_speed_kp <- {val}")
                continue
            if cmd == "ki" and len(parts) == 2:
                val = float(parts[1])
                ctrl.write_reg("pid_speed_ki", val)
                print(f"pid_speed_ki <- {val}")
                continue
            if cmd == "speed" and len(parts) == 2:
                val = float(parts[1])
                ctrl.write_reg("cal_speed", val)
                print(f"cal_speed <- {val}")
                continue
            if cmd == "state" and len(parts) == 2:
                val = int(parts[1], 0)
                ctrl.write_reg("state", val)
                print(f"state <- {val}")
                continue
            if cmd == "save":
                ctrl.write_reg("save_conf", 1)
                print("save_conf <- 1")
                continue
            if cmd == "stop":
                ctrl.write_reg("cal_speed", 0.0)
                ctrl.write_reg("state", 0)
                print("motor stopped")
                continue
            if cmd == "start":
                ctrl.write_reg("state", 3)
                print("speed mode started")
                continue
            if cmd == "show":
                print(
                    "cal_pos={cal_pos} sen_pos={sen_pos}".format(
                        cal_pos=ctrl.read_reg("cal_pos"),
                        sen_pos=ctrl.read_reg("sen_pos"),
                    )
                )
                continue

            print("invalid command")
            print_runtime_help()

    except KeyboardInterrupt:
        print("\nkeyboard interrupted")
    finally:
        stop_event.set()
        t.join(timeout=1.0)
        if not args.keep_running:
            try:
                ctrl.write_reg("cal_speed", 0.0)
                ctrl.write_reg("state", 0)
                print("exit cleanup: motor stopped")
            except Exception as e:
                print(f"exit cleanup warning: {e}")


if __name__ == "__main__":
    main()
