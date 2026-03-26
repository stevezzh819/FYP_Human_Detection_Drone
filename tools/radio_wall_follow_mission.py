#!/usr/bin/env python3
"""Start the onboard wall-follow mission over Crazyradio and stream telemetry.

This script does not send velocity commands. It only enables the app-layer
mission by setting `app.active=1`, then subscribes to a few useful log blocks:
  - Flow deck motion: motion.deltaX, motion.deltaY, motion.shutter
  - Multiranger distances: range.front/left/right/back
  - Mission / ESP32-derived state: app.mission, app.human*, app.cmdVel*

If `--esp-port` is provided and pyserial is installed, the script also tails the
ESP32 USB serial output in a background thread. That is suitable for tethered
bench tests. For free flight, prefer the Crazyflie radio logs.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import usb.core
import usb.backend.libusb1

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    serial = None


@dataclass
class MissionState:
    mission_started: bool = False
    mission_finished: bool = False
    state_outer: int = 0
    mission: int = 0
    human_hold: float = 0.0
    stop_requested: bool = False


def check_crazyradio_access() -> tuple[bool, str]:
    backend = usb.backend.libusb1.get_backend()
    dev = usb.core.find(idVendor=0x1915, idProduct=0x7777, backend=backend)
    if dev is None:
        return False, "Crazyradio dongle not detected on USB"

    try:
        dev.set_configuration(1)
    except usb.core.USBError as exc:
        if exc.errno == 19:
            return False, (
                "Crazyradio is visible to macOS but libusb cannot open it "
                "(USB Errno 19). Replug the dongle, try another USB port, and "
                "close any app that may be using it."
            )
        return False, f"Crazyradio open failed: {exc}"

    return True, "Crazyradio USB access is working"


def discover_uri(preferred: Optional[str]) -> str:
    if preferred:
        return preferred

    try:
        links = cflib.crtp.scan_interfaces()
    except Exception as exc:
        raise RuntimeError(f"Failed to scan Crazyflie interfaces: {exc}") from exc

    if not links:
        raise RuntimeError("No Crazyflie links found. Plug in the Crazyradio and power the drone.")

    for uri, _ in links:
        if uri.startswith("radio://"):
            return uri

    return links[0][0]


def start_esp_monitor(port: str, baudrate: int, state: MissionState) -> threading.Thread:
    if serial is None:
        raise RuntimeError("pyserial is not installed, cannot open --esp-port")

    def run() -> None:
        try:
            with serial.Serial(port, baudrate=baudrate, timeout=0.25) as ser:
                print(f"[esp] monitoring {port} @ {baudrate}")
                while not state.stop_requested:
                    line = ser.readline()
                    if not line:
                        continue
                    print(f"[esp] {line.decode(errors='replace').rstrip()}")
        except Exception as exc:  # pragma: no cover - hardware dependent
            print(f"[esp] monitor stopped: {exc}", file=sys.stderr)

    thread = threading.Thread(target=run, name="esp-monitor", daemon=True)
    thread.start()
    return thread


def main() -> int:
    parser = argparse.ArgumentParser(description="Start onboard wall-follow mission over Crazyradio")
    parser.add_argument("--uri", help="Crazyflie link URI. If omitted, auto-discover over radio.")
    parser.add_argument("--cache", default="/Users/zhangzehua/Desktop/fyp/cache", help="TOC cache directory")
    parser.add_argument("--start-delay", type=float, default=2.0, help="Seconds to wait before setting app.active=1")
    parser.add_argument("--max-duration", type=float, default=90.0, help="Safety timeout in seconds")
    parser.add_argument("--log-period-ms", type=int, default=250, help="Telemetry log period in ms")
    parser.add_argument("--esp-port", help="Optional ESP32 serial port for tethered monitor, e.g. /dev/cu.usbmodem1101")
    parser.add_argument("--esp-baud", type=int, default=115200, help="ESP32 monitor baud rate")
    parser.add_argument("--auto-start", action="store_true", help="Start immediately after connecting, without an interactive prompt")
    parser.add_argument("--check-radio", action="store_true", help="Only verify Crazyradio USB access and exit")
    parser.add_argument("--dry-run", action="store_true", help="Only discover the Crazyflie link URI and exit")
    args = parser.parse_args()

    if args.check_radio:
        ok, message = check_crazyradio_access()
        stream = sys.stdout if ok else sys.stderr
        print(f"[radio] {message}", file=stream)
        return 0 if ok else 1

    cflib.crtp.init_drivers()
    try:
        uri = discover_uri(args.uri)
    except RuntimeError as exc:
        ok, radio_message = check_crazyradio_access()
        if not ok:
            print(f"[radio] {radio_message}", file=sys.stderr)
        else:
            print(f"[radio] {exc}", file=sys.stderr)
            print("[radio] Crazyradio is accessible, but no powered Crazyflie responded.", file=sys.stderr)
        return 1

    if args.dry_run:
        print(f"[radio] discovered {uri}")
        return 0

    cf = Crazyflie(rw_cache=args.cache)
    state = MissionState()
    logconfs: list[LogConfig] = []

    def request_stop(_sig: int, _frame: object) -> None:
        state.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGTSTP, request_stop)

    if args.esp_port:
        start_esp_monitor(args.esp_port, args.esp_baud, state)

    line_buf: list[str] = []

    def on_console(chars: str) -> None:
        for ch in chars:
            if ch == "\n":
                line = "".join(line_buf).strip("\r")
                line_buf.clear()
                if line:
                    print(f"[cf-console] {line}")
            else:
                line_buf.append(ch)

    def mission_log(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
        state.state_outer = int(data["app.stateOuter"])
        state.mission = int(data["app.mission"])
        state.human_hold = float(data["app.humanHold"])
        human = int(data["app.human"])
        conf = int(data["app.humanConf"])
        direction = int(data["app.humanDir"])
        fresh = int(data["app.humanFresh"])
        stable = int(data["app.humanStable"])
        z = float(data["stateEstimate.z"])
        max_temp = int(data["app.humanMax"]) / 100.0
        therm = int(data["app.humanTherm"]) / 100.0

        if state.state_outer == 1:
            state.mission_started = True
        elif state.mission_started and state.state_outer == 0:
            state.mission_finished = True

        print(
            "[mission] "
            f"t={ts} outer={state.state_outer} mission={state.mission} "
            f"z={z:.2f} "
            f"human={human} fresh={fresh} stable={stable} hold={state.human_hold:.2f}s "
            f"conf={conf} dir={direction} max={max_temp:.2f}C therm={therm:.2f}C"
        )

    def flow_log(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
        dx = int(data["motion.deltaX"])
        dy = int(data["motion.deltaY"])
        shutter = int(data["motion.shutter"])
        vx = float(data["stateEstimate.vx"])
        vy = float(data["stateEstimate.vy"])
        cmd_vx = float(data["app.cmdVelX"])
        cmd_vy = float(data["app.cmdVelY"])
        print(
            f"[flow] t={ts} delta=({dx},{dy}) shutter={shutter} "
            f"vel=({vx:.2f},{vy:.2f}) cmd=({cmd_vx:.2f},{cmd_vy:.2f})"
        )

    def range_log(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
        front = int(data["range.front"])
        left = int(data["range.left"])
        right = int(data["range.right"])
        back = int(data["range.back"])
        up = int(data["range.up"])
        vbat = float(data["pm.vbat"])
        print(
            f"[range] t={ts} front={front} left={left} right={right} back={back} up={up} vbat={vbat:.2f}"
        )

    print(f"[radio] connecting to {uri}")

    with SyncCrazyflie(uri, cf=cf):
        mission_cfg = LogConfig(name="mission", period_in_ms=args.log_period_ms)
        mission_cfg.add_variable("app.stateOuter", "uint8_t")
        mission_cfg.add_variable("app.mission", "uint8_t")
        mission_cfg.add_variable("app.human", "uint8_t")
        mission_cfg.add_variable("app.humanConf", "uint8_t")
        mission_cfg.add_variable("app.humanDir", "int8_t")
        mission_cfg.add_variable("app.humanFresh", "uint8_t")
        mission_cfg.add_variable("app.humanStable", "uint8_t")
        mission_cfg.add_variable("app.humanHold", "float")
        mission_cfg.add_variable("app.humanMax", "int16_t")
        mission_cfg.add_variable("app.humanTherm", "int16_t")
        mission_cfg.add_variable("stateEstimate.z", "float")
        mission_cfg.data_received_cb.add_callback(mission_log)
        cf.log.add_config(mission_cfg)

        flow_cfg = LogConfig(name="flow", period_in_ms=args.log_period_ms)
        flow_cfg.add_variable("motion.deltaX", "int16_t")
        flow_cfg.add_variable("motion.deltaY", "int16_t")
        flow_cfg.add_variable("motion.shutter", "uint16_t")
        flow_cfg.add_variable("stateEstimate.vx", "float")
        flow_cfg.add_variable("stateEstimate.vy", "float")
        flow_cfg.add_variable("app.cmdVelX", "float")
        flow_cfg.add_variable("app.cmdVelY", "float")
        flow_cfg.data_received_cb.add_callback(flow_log)
        cf.log.add_config(flow_cfg)

        range_cfg = LogConfig(name="range", period_in_ms=args.log_period_ms)
        range_cfg.add_variable("range.front", "uint16_t")
        range_cfg.add_variable("range.left", "uint16_t")
        range_cfg.add_variable("range.right", "uint16_t")
        range_cfg.add_variable("range.back", "uint16_t")
        range_cfg.add_variable("range.up", "uint16_t")
        range_cfg.add_variable("pm.vbat", "float")
        range_cfg.data_received_cb.add_callback(range_log)
        cf.log.add_config(range_cfg)

        try:
            cf.param.set_value("app.active", "0")
            time.sleep(0.5)
            if not args.auto_start:
                if not sys.stdin.isatty():
                    print("[radio] stdin is not interactive; rerun with --auto-start to launch without a prompt", file=sys.stderr)
                    return 2

                print("[radio] Waiting for explicit start confirmation")
                try:
                    answer = input("[radio] Type FLY and press Enter to start the mission\n> ").strip()
                except EOFError:
                    print("[radio] start aborted: stdin closed")
                    return 0
                if answer.upper() != "FLY":
                    print("[radio] start aborted by user")
                    return 0

            cf.console.receivedChar.add_callback(on_console)
            logconfs.extend([mission_cfg, flow_cfg, range_cfg])
            for cfg in logconfs:
                cfg.start()

            print(f"[radio] starting mission in {args.start_delay:.1f}s")
            time.sleep(args.start_delay)
            cf.param.set_value("app.active", "1")
            print("[radio] app.active=1")

            deadline = time.time() + args.max_duration
            while time.time() < deadline and not state.stop_requested:
                if state.mission_finished:
                    print("[radio] mission finished and Crazyflie returned to idle")
                    break
                time.sleep(0.1)
            else:
                print("[radio] safety timeout reached, disabling app.active")
        finally:
            try:
                cf.param.set_value("app.active", "0")
            except Exception:
                pass
            for cfg in logconfs:
                try:
                    cfg.stop()
                except Exception:
                    pass

    state.stop_requested = True
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
