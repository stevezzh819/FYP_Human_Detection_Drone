#!/usr/bin/env python3
"""Tk dashboard for the Crazyflie wall-follow mission over Crazyradio."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from radio_wall_follow_mission import check_crazyradio_access, discover_uri


OUTER_NAMES = {
    0: "Idle",
    1: "Unlocked",
    2: "Stopping",
}

MISSION_NAMES = {
    0: "Reacquire Wall",
    1: "Wall Follow",
    2: "Scan",
    3: "Signal",
    4: "Land",
    5: "Transition",
}

DIR_NAMES = {
    -1: "Left",
    0: "Center",
    1: "Right",
}

PALETTE = {
    "app_bg": "#080808",
    "panel_bg": "#121212",
    "panel_edge": "#2c2c2c",
    "text_main": "#f5f5f5",
    "text_muted": "#b8b8b8",
    "text_dim": "#8a8a8a",
    "button_bg": "#1d1d1d",
    "button_active": "#34c82a",
    "field_bg": "#151515",
    "canvas_bg": "#0d0d0d",
    "sector_far": "#F127EE",
    "sector_mid": "#B867AB",
    "sector_near": "#34c82a",
    "human_off": "#353535",
    "human_fresh": "#a7a7a7",
    "human_stable": "#e81717",
}


@dataclass
class MissionSnapshot:
    timestamp_ms: int = 0
    outer: int = 0
    mission: int = 0
    z: float = 0.0
    human: int = 0
    fresh: int = 0
    stable: int = 0
    hold: float = 0.0
    conf: int = 0
    direction: int = 0
    max_temp_c: float = 0.0
    therm_c: float = 0.0


@dataclass
class FlowSnapshot:
    timestamp_ms: int = 0
    delta_x: int = 0
    delta_y: int = 0
    shutter: int = 0
    vel_x: float = 0.0
    vel_y: float = 0.0
    cmd_x: float = 0.0
    cmd_y: float = 0.0
    pos_x_cm: float = 0.0
    pos_y_cm: float = 0.0
    pos_z_cm: float = 0.0
    pos_valid: int = 0


@dataclass
class RangeSnapshot:
    timestamp_ms: int = 0
    front: int = 32766
    left: int = 32766
    right: int = 32766
    back: int = 32766
    up: int = 32766
    vbat: float = 0.0


class TelemetryCsvLogger:
    FIELDNAMES = [
        "Time",
        "uri",
        "source",
        "outer",
        "mission",
        "mission_name",
        "shutter",
        "human",
        "human_fresh",
        "human_stable",
        "human_hold_s",
        "human_confidence_pct",
        "human_direction",
        "human_max_temp_c",
        "human_thermistor_c",
        "flow_timestamp_ms",
        "delta_x",
        "delta_y",
        "pos_x_cm",
        "pos_y_cm",
        "height_m",
        "vel_x_mps",
        "vel_y_mps",
        "cmd_x_mps",
        "cmd_y_mps",
        "front_mm",
        "left_mm",
        "right_mm",
        "back_mm",
        "up_mm",
        "vbat_v",
    ]

    def __init__(self, csv_dir: str, uri: str) -> None:
        self.uri = uri
        self.latest_mission = MissionSnapshot()
        self.latest_flow = FlowSnapshot()
        self.latest_range = RangeSnapshot()

        safe_uri = "".join(ch if ch.isalnum() else "_" for ch in uri).strip("_") or "unknown_uri"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = pathlib.Path(csv_dir).expanduser()
        base_dir.mkdir(parents=True, exist_ok=True)
        self.path = base_dir / f"wall_follow_telemetry_{timestamp}_{safe_uri}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()

    def log_mission(self, snapshot: MissionSnapshot) -> None:
        self.latest_mission = snapshot
        self._write_row(source="mission")

    def log_flow(self, snapshot: FlowSnapshot) -> None:
        self.latest_flow = snapshot
        self._write_row(source="flow")

    def log_range(self, snapshot: RangeSnapshot) -> None:
        self.latest_range = snapshot
        self._write_row(source="range")

    def _write_row(self, *, source: str) -> None:
        now = time.time()
        self._writer.writerow(
            {
                "Time": datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
                "uri": self.uri,
                "source": source,
                "outer": self.latest_mission.outer,
                "mission": self.latest_mission.mission,
                "mission_name": MISSION_NAMES.get(self.latest_mission.mission, str(self.latest_mission.mission)),
                "height_m": self.latest_mission.z,
                "human": self.latest_mission.human,
                "human_fresh": self.latest_mission.fresh,
                "human_stable": self.latest_mission.stable,
                "human_hold_s": self.latest_mission.hold,
                "human_confidence_pct": self.latest_mission.conf,
                "human_direction": self.latest_mission.direction,
                "human_max_temp_c": self.latest_mission.max_temp_c,
                "human_thermistor_c": self.latest_mission.therm_c,
                "flow_timestamp_ms": self.latest_flow.timestamp_ms,
                "delta_x": self.latest_flow.delta_x,
                "delta_y": self.latest_flow.delta_y,
                "pos_x_cm": self.latest_flow.pos_x_cm,
                "pos_y_cm": self.latest_flow.pos_y_cm,
                "shutter": self.latest_flow.shutter,
                "vel_x_mps": self.latest_flow.vel_x,
                "vel_y_mps": self.latest_flow.vel_y,
                "cmd_x_mps": self.latest_flow.cmd_x,
                "cmd_y_mps": self.latest_flow.cmd_y,
                "front_mm": self.latest_range.front,
                "left_mm": self.latest_range.left,
                "right_mm": self.latest_range.right,
                "back_mm": self.latest_range.back,
                "up_mm": self.latest_range.up,
                "vbat_v": self.latest_range.vbat,
            }
        )
        self._file.flush()


class ColorButton(tk.Label):
    def __init__(
        self,
        parent: tk.Widget,
        *,
        text: str,
        command,
        bg: str,
        fg: str,
        active_bg: str,
        active_fg: str,
        disabled_bg: str = "#2a2a2a",
        disabled_fg: str = "#7a7a7a",
        width: int = 12,
        font = ("Helvetica", 11, "bold"),
    ) -> None:
        super().__init__(
            parent,
            text=text,
            bg=bg,
            fg=fg,
            width=width,
            font=font,
            padx=12,
            pady=8,
            bd=0,
            relief="flat",
            cursor="hand2",
        )
        self.command = command
        self.base_bg = bg
        self.base_fg = fg
        self.active_bg = active_bg
        self.active_fg = active_fg
        self.disabled_bg = disabled_bg
        self.disabled_fg = disabled_fg
        self.enabled = True

        self.bind("<Button-1>", self._on_click)
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)

    def set_state(self, state: str) -> None:
        self.enabled = state != "disabled"
        self.configure(
            bg=self.base_bg if self.enabled else self.disabled_bg,
            fg=self.base_fg if self.enabled else self.disabled_fg,
            cursor="hand2" if self.enabled else "arrow",
        )

    def _on_click(self, _event) -> None:
        if self.enabled:
            self.command()

    def _on_enter(self, _event) -> None:
        if self.enabled:
            self.configure(bg=self.active_bg, fg=self.active_fg)

    def _on_leave(self, _event) -> None:
        if self.enabled:
            self.configure(bg=self.base_bg, fg=self.base_fg)


def human_led_color(snapshot: MissionSnapshot) -> str:
    if snapshot.human and snapshot.fresh and snapshot.stable:
        return PALETTE["human_stable"]
    if snapshot.human and snapshot.fresh:
        return PALETTE["human_fresh"]
    return PALETTE["human_off"]


def range_fill(distance_mm: int) -> str:
    if distance_mm >= 32000:
        return PALETTE["sector_far"]
    if distance_mm < 120:
        return PALETTE["sector_near"]
    if distance_mm < 250:
        return PALETTE["sector_mid"]
    return PALETTE["sector_far"]


def pretty_distance(distance_mm: int) -> str:
    if distance_mm >= 32000:
        return "--"
    return f"{distance_mm / 10.0:.1f} cm"


def summarize_link_error(exc: Exception, uri: str) -> str:
    message = str(exc)
    if "\n\nTraceback" in message:
        message = message.split("\n\nTraceback", 1)[0].strip()

    if "Too many packets lost" in message:
        return (
            f"{message}. Move the Crazyflie closer to the Crazyradio, "
            "verify the battery is connected, and confirm the selected radio URI."
        )

    if "[Errno 19]" in message and uri.startswith("radio://"):
        return (
            "Crazyradio is visible but libusb could not open it. "
            "Replug the Crazyradio and close any other app that may be using it."
        )

    if "Could not open usb://0" in message or (uri.startswith("usb://") and "Access denied" in message):
        return (
            "Crazyflie USB link could not be opened. "
            "Replug the Crazyflie USB cable and close any other app using the USB link."
        )

    return message


class RadioSession(threading.Thread):
    def __init__(
        self,
        preferred_uri: Optional[str],
        cache_dir: str,
        csv_dir: str,
        log_period_ms: int,
        event_queue: "queue.Queue[tuple[str, Dict[str, object]]]",
    ) -> None:
        super().__init__(name="cf-radio-session", daemon=True)
        self.preferred_uri = preferred_uri
        self.cache_dir = cache_dir
        self.csv_dir = csv_dir
        self.log_period_ms = log_period_ms
        self.event_queue = event_queue
        self.command_queue: "queue.Queue[tuple[str, Optional[int]]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.cf: Optional[Crazyflie] = None

    def request_active(self, active: bool) -> None:
        self.command_queue.put(("set_active", 1 if active else 0))

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.cf is not None:
            try:
                self.cf.close_link()
            except Exception:
                pass
        self.command_queue.put(("stop", None))

    def _emit(self, event_type: str, **payload: object) -> None:
        self.event_queue.put((event_type, payload))

    def run(self) -> None:
        cflib.crtp.init_drivers()

        try:
            uri = discover_uri(self.preferred_uri)
        except RuntimeError as exc:
            self._emit("status", level="error", message=str(exc))
            self._emit("disconnected")
            return

        cf = Crazyflie(rw_cache=self.cache_dir)
        self.cf = cf
        csv_logger: Optional[TelemetryCsvLogger] = None
        logconfs: list[LogConfig] = []
        line_buffer: list[str] = []
        latest_flow = FlowSnapshot()
        latest_world_x = 0.0
        latest_world_y = 0.0
        latest_world_z = 0.0
        latest_yaw_deg = 0.0
        origin_capture_pending = False
        origin_world_x: Optional[float] = None
        origin_world_y: Optional[float] = None
        origin_world_z: Optional[float] = None
        origin_yaw_deg: Optional[float] = None

        def on_console(chars: str) -> None:
            for ch in chars:
                if ch == "\n":
                    line = "".join(line_buffer).strip("\r")
                    line_buffer.clear()
                    if line:
                        self._emit("console", line=line)
                else:
                    line_buffer.append(ch)

        def on_log_error(logconf: LogConfig, message: str) -> None:
            self._emit("status", level="error", message=f"{logconf.name}: {message}")

        def on_mission(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            snapshot = MissionSnapshot(
                timestamp_ms=ts,
                outer=int(data["app.stateOuter"]),
                mission=int(data["app.mission"]),
                human=int(data["app.human"]),
                fresh=int(data["app.humanFresh"]),
                stable=int(data["app.humanStable"]),
                hold=float(data["app.humanHold"]),
                conf=int(data["app.humanConf"]),
                direction=int(data["app.humanDir"]),
                z=float(data["stateEstimate.z"]),
                max_temp_c=int(data["app.humanMax"]) / 100.0,
                therm_c=int(data["app.humanTherm"]) / 100.0,
            )
            if csv_logger is not None:
                csv_logger.log_mission(snapshot)
            self._emit("mission", **snapshot.__dict__)

        def emit_flow_snapshot() -> None:
            nonlocal latest_flow
            snapshot = FlowSnapshot(**latest_flow.__dict__)
            if csv_logger is not None:
                csv_logger.log_flow(snapshot)
            self._emit("flow", **snapshot.__dict__)

        def update_relative_position() -> None:
            nonlocal origin_capture_pending
            nonlocal origin_world_x
            nonlocal origin_world_y
            nonlocal origin_world_z
            nonlocal origin_yaw_deg
            nonlocal latest_flow

            if origin_capture_pending and origin_world_x is None:
                origin_world_x = latest_world_x
                origin_world_y = latest_world_y
                origin_world_z = latest_world_z
                origin_yaw_deg = latest_yaw_deg
                origin_capture_pending = False

            if origin_world_x is None or origin_world_y is None or origin_world_z is None or origin_yaw_deg is None:
                latest_flow.pos_valid = 0
                latest_flow.pos_x_cm = 0.0
                latest_flow.pos_y_cm = 0.0
                latest_flow.pos_z_cm = 0.0
                return

            dx = latest_world_x - origin_world_x
            dy = latest_world_y - origin_world_y
            dz = latest_world_z - origin_world_z
            yaw0_rad = math.radians(origin_yaw_deg)

            forward_m = (dx * math.cos(yaw0_rad)) + (dy * math.sin(yaw0_rad))
            left_m = (-dx * math.sin(yaw0_rad)) + (dy * math.cos(yaw0_rad))

            latest_flow.pos_x_cm = -left_m * 100.0
            latest_flow.pos_y_cm = forward_m * 100.0
            latest_flow.pos_z_cm = dz * 100.0
            latest_flow.pos_valid = 1

        def on_flow(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            nonlocal latest_flow
            latest_flow.timestamp_ms = ts
            latest_flow.delta_x = int(data["motion.deltaX"])
            latest_flow.delta_y = int(data["motion.deltaY"])
            latest_flow.shutter = int(data["motion.shutter"])
            latest_flow.vel_x = float(data["stateEstimate.vx"])
            latest_flow.vel_y = float(data["stateEstimate.vy"])
            latest_flow.cmd_x = float(data["app.cmdVelX"])
            latest_flow.cmd_y = float(data["app.cmdVelY"])
            update_relative_position()
            emit_flow_snapshot()

        def on_pose(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            nonlocal latest_flow
            nonlocal latest_world_x
            nonlocal latest_world_y
            nonlocal latest_world_z
            nonlocal latest_yaw_deg
            latest_flow.timestamp_ms = ts
            latest_world_x = float(data["stateEstimate.x"])
            latest_world_y = float(data["stateEstimate.y"])
            latest_world_z = float(data["stateEstimate.z"])
            latest_yaw_deg = float(data["stabilizer.yaw"])
            update_relative_position()
            emit_flow_snapshot()

        def on_range(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            snapshot = RangeSnapshot(
                timestamp_ms=ts,
                front=int(data["range.front"]),
                left=int(data["range.left"]),
                right=int(data["range.right"]),
                back=int(data["range.back"]),
                up=int(data["range.up"]),
                vbat=float(data["pm.vbat"]),
            )
            if csv_logger is not None:
                csv_logger.log_range(snapshot)
            self._emit("range", **snapshot.__dict__)

        def set_active_with_retry(active: int) -> None:
            last_error: Optional[Exception] = None
            for _ in range(20):
                try:
                    if cf.param.toc.get_element("app", "active") is None:
                        time.sleep(0.2)
                        continue
                    cf.param.set_value("app.active", str(active))
                    return
                except KeyError as exc:
                    last_error = exc
                    time.sleep(0.2)
                except Exception as exc:
                    last_error = exc
                    break

            if last_error is not None:
                self._emit("status", level="error", message=f"Failed to set app.active={active}: {last_error}")

        self._emit("status", level="info", message=f"Connecting to {uri}")

        try:
            with SyncCrazyflie(uri, cf=cf) as scf:
                scf.wait_for_params()
                try:
                    csv_logger = TelemetryCsvLogger(self.csv_dir, uri)
                    self._emit("status", level="info", message=f"CSV telemetry logging to {csv_logger.path}")
                except Exception as exc:
                    self._emit("status", level="error", message=f"Failed to start CSV telemetry logging: {exc}")

                mission_cfg = LogConfig(name="mission", period_in_ms=self.log_period_ms)
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
                mission_cfg.data_received_cb.add_callback(on_mission)
                mission_cfg.error_cb.add_callback(on_log_error)
                cf.log.add_config(mission_cfg)

                flow_cfg = LogConfig(name="flow", period_in_ms=self.log_period_ms)
                flow_cfg.add_variable("motion.deltaX", "int16_t")
                flow_cfg.add_variable("motion.deltaY", "int16_t")
                flow_cfg.add_variable("motion.shutter", "uint16_t")
                flow_cfg.add_variable("stateEstimate.vx", "float")
                flow_cfg.add_variable("stateEstimate.vy", "float")
                flow_cfg.add_variable("app.cmdVelX", "float")
                flow_cfg.add_variable("app.cmdVelY", "float")
                flow_cfg.data_received_cb.add_callback(on_flow)
                flow_cfg.error_cb.add_callback(on_log_error)
                cf.log.add_config(flow_cfg)

                pose_cfg = LogConfig(name="pose", period_in_ms=self.log_period_ms)
                pose_cfg.add_variable("stateEstimate.x", "float")
                pose_cfg.add_variable("stateEstimate.y", "float")
                pose_cfg.add_variable("stateEstimate.z", "float")
                pose_cfg.add_variable("stabilizer.yaw", "float")
                pose_cfg.data_received_cb.add_callback(on_pose)
                pose_cfg.error_cb.add_callback(on_log_error)
                cf.log.add_config(pose_cfg)

                range_cfg = LogConfig(name="range", period_in_ms=self.log_period_ms)
                range_cfg.add_variable("range.front", "uint16_t")
                range_cfg.add_variable("range.left", "uint16_t")
                range_cfg.add_variable("range.right", "uint16_t")
                range_cfg.add_variable("range.back", "uint16_t")
                range_cfg.add_variable("range.up", "uint16_t")
                range_cfg.add_variable("pm.vbat", "float")
                range_cfg.data_received_cb.add_callback(on_range)
                range_cfg.error_cb.add_callback(on_log_error)
                cf.log.add_config(range_cfg)

                logconfs.extend([mission_cfg, flow_cfg, pose_cfg, range_cfg])
                for cfg in logconfs:
                    cfg.start()

                cf.console.receivedChar.add_callback(on_console)
                set_active_with_retry(0)
                self._emit("connected", uri=uri)
                self._emit("status", level="info", message="Telemetry running. Use Start Mission to arm the app.")

                while not self.stop_event.is_set():
                    try:
                        command, value = self.command_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    if command == "set_active":
                        origin_capture_pending = int(value or 0) == 1
                        if origin_capture_pending:
                            origin_world_x = None
                            origin_world_y = None
                            origin_world_z = None
                            origin_yaw_deg = None
                            latest_flow.pos_valid = 0
                            latest_flow.pos_x_cm = 0.0
                            latest_flow.pos_y_cm = 0.0
                            latest_flow.pos_z_cm = 0.0
                        set_active_with_retry(int(value or 0))
                        self._emit("status", level="info", message=f"app.active={int(value or 0)}")
                    elif command == "stop":
                        break
        except Exception as exc:
            self._emit("status", level="error", message=summarize_link_error(exc, uri))
        finally:
            try:
                set_active_with_retry(0)
            except Exception:
                pass

            for cfg in logconfs:
                try:
                    cfg.stop()
                except Exception:
                    pass

            if csv_logger is not None:
                try:
                    csv_logger.close()
                except Exception:
                    pass

            self.cf = None
            self._emit("disconnected")


class ScanSession(threading.Thread):
    def __init__(self, event_queue: "queue.Queue[tuple[str, Dict[str, object]]]") -> None:
        super().__init__(name="cf-scan-session", daemon=True)
        self.event_queue = event_queue

    def _emit(self, event_type: str, **payload: object) -> None:
        self.event_queue.put((event_type, payload))

    def run(self) -> None:
        cflib.crtp.init_drivers()

        try:
            links = cflib.crtp.scan_interfaces()
        except Exception as exc:
            ok, radio_message = check_crazyradio_access()
            if not ok:
                self._emit("status", level="error", message=radio_message)
            else:
                self._emit("status", level="error", message=f"Failed to scan Crazyflie interfaces: {exc}")
            self._emit("scan_result", interfaces=[], selected_uri=None)
            return

        interfaces = [uri for uri, _ in links]
        selected_uri = None
        for uri in interfaces:
            if uri.startswith("radio://"):
                selected_uri = uri
                break
        if selected_uri is None and interfaces:
            selected_uri = interfaces[0]

        self._emit("scan_result", interfaces=interfaces, selected_uri=selected_uri)


class DashboardApp:
    STATE_DISCONNECTED = "disconnected"
    STATE_CONNECTING = "connecting"
    STATE_CONNECTED = "connected"
    STATE_SCANNING = "scanning"

    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.event_queue: "queue.Queue[tuple[str, Dict[str, object]]]" = queue.Queue()
        self.worker: Optional[RadioSession] = None
        self.scan_worker: Optional[ScanSession] = None
        self.connected = False
        self.closing = False
        self.pending_start = False
        self.ui_state = self.STATE_DISCONNECTED
        self.last_scan_uris: list[str] = []
        self.mission = MissionSnapshot()
        self.flow = FlowSnapshot()
        self.range = RangeSnapshot()
        self.trace_points: list[tuple[float, float, float]] = []
        self.trace_active = False
        self.path_breath_phase = 0.0
        self.console_lines: list[str] = []
        self.uri_var = tk.StringVar(value=args.uri or "usb://0")
        self.status_var = tk.StringVar(value="Waiting to connect")
        self.connection_var = tk.StringVar(value="Disconnected")

        self._configure_root()
        self._build_layout()
        self._render_all()
        self._set_ui_state(self.STATE_DISCONNECTED)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(60, self._pump_events)
        self.root.after(80, self._animate_path_head)
        if self.args.auto_connect:
            self.root.after(150, self.connect)

    def _configure_root(self) -> None:
        self.root.title("Crazyflie Mission Dashboard")
        self.root.geometry("1280x960")
        self.root.minsize(1160, 860)
        self.root.configure(bg=PALETTE["app_bg"])

    def _panel(self, parent: tk.Widget, title: str) -> tk.Frame:
        frame = tk.Frame(parent, bg=PALETTE["panel_bg"], bd=0, highlightthickness=1, highlightbackground=PALETTE["panel_edge"])
        header = tk.Label(
            frame,
            text=title,
            bg=PALETTE["panel_bg"],
            fg=PALETTE["text_main"],
            font=("Helvetica", 14, "bold"),
            anchor="w",
        )
        header.pack(fill="x", padx=16, pady=(14, 8))
        return frame

    def _value_pair(self, parent: tk.Widget, row: int, label: str) -> tk.Label:
        tk.Label(
            parent,
            text=label,
            bg=PALETTE["panel_bg"],
            fg=PALETTE["text_dim"],
            font=("Helvetica", 11),
            anchor="w",
        ).grid(row=row, column=0, sticky="w", padx=(16, 10), pady=4)
        value = tk.Label(
            parent,
            text="--",
            bg=PALETTE["panel_bg"],
            fg=PALETTE["text_main"],
            font=("Helvetica", 12, "bold"),
            anchor="w",
        )
        value.grid(row=row, column=1, sticky="w", padx=(0, 16), pady=4)
        return value

    def _build_layout(self) -> None:
        top_bar = tk.Frame(self.root, bg=PALETTE["app_bg"])
        top_bar.pack(fill="x", padx=18, pady=(18, 12))

        control_row = tk.Frame(top_bar, bg=PALETTE["app_bg"])
        control_row.pack(fill="x")
        tk.Label(
            control_row,
            text="URI",
            bg=PALETTE["app_bg"],
            fg=PALETTE["text_dim"],
            font=("Helvetica", 11, "bold"),
        ).pack(side="left", padx=(0, 8))
        self.uri_entry = tk.Entry(
            control_row,
            textvariable=self.uri_var,
            bg=PALETTE["field_bg"],
            fg=PALETTE["text_main"],
            insertbackground=PALETTE["text_main"],
            relief="flat",
            highlightthickness=1,
            highlightbackground=PALETTE["panel_edge"],
            font=("Menlo", 12),
            width=24,
        )
        self.uri_entry.pack(side="left", ipady=8)

        buttons = tk.Frame(control_row, bg=PALETTE["app_bg"])
        buttons.pack(side="right")
        self.scan_button = ColorButton(
            buttons,
            text="Scan",
            command=self.scan,
            bg="#2b2b2b",
            fg="#ffffff",
            active_bg="#1f1f1f",
            active_fg="#ffffff",
            width=10,
            font=("Helvetica", 11, "bold"),
        )
        self.scan_button.grid(row=0, column=0, padx=4)
        self.connect_button = ColorButton(
            buttons,
            text="Connect",
            command=self.connect,
            bg="#000000",
            fg="#ffffff",
            active_bg="#1a1a1a",
            active_fg="#ffffff",
            width=10,
            font=("Helvetica", 11, "bold"),
        )
        self.connect_button.grid(row=0, column=1, padx=4)
        self.start_button = ColorButton(
            buttons,
            text="Start Mission",
            command=self.start_mission,
            bg="#19fd20",
            fg="#000000",
            active_bg="#10df16",
            active_fg="#000000",
            width=12,
            font=("Helvetica", 11, "bold"),
        )
        self.start_button.set_state("disabled")
        self.start_button.grid(row=0, column=2, padx=4)
        self.stop_button = ColorButton(
            buttons,
            text="Stop Mission",
            command=self.stop_mission,
            bg="#ff3b30",
            fg="#000000",
            active_bg="#e3342a",
            active_fg="#000000",
            width=12,
            font=("Helvetica", 11, "bold"),
        )
        self.stop_button.set_state("disabled")
        self.stop_button.grid(row=0, column=3, padx=4)

        status_row = tk.Frame(self.root, bg=PALETTE["app_bg"])
        status_row.pack(fill="x", padx=18, pady=(0, 12))
        tk.Label(status_row, text="Connection", bg=PALETTE["app_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11, "bold")).pack(side="left")
        tk.Label(status_row, textvariable=self.connection_var, bg=PALETTE["app_bg"], fg=PALETTE["text_main"], font=("Helvetica", 11)).pack(side="left", padx=(10, 24))
        tk.Label(status_row, textvariable=self.status_var, bg=PALETTE["app_bg"], fg=PALETTE["text_muted"], font=("Helvetica", 11), anchor="w").pack(side="left")

        cards = tk.Frame(self.root, bg=PALETTE["app_bg"])
        cards.pack(fill="both", expand=True, padx=18, pady=(0, 18))
        cards.grid_columnconfigure(0, weight=1)
        cards.grid_columnconfigure(1, weight=1)
        cards.grid_columnconfigure(2, weight=1)
        cards.grid_columnconfigure(3, weight=1)
        cards.grid_rowconfigure(0, weight=3)
        cards.grid_rowconfigure(1, weight=2)

        self.mission_frame = self._panel(cards, "Mission")
        self.mission_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 9))
        self._build_mission_panel()

        self.human_frame = self._panel(cards, "Human Detection")
        self.human_frame.grid(row=0, column=1, sticky="nsew", padx=6, pady=(0, 9))
        self._build_human_panel()

        self.path_frame = self._panel(cards, "Flying Path")
        self.path_frame.grid(row=0, column=2, columnspan=2, sticky="nsew", padx=(6, 0), pady=(0, 9))
        self._build_path_panel()

        self.flow_frame = self._panel(cards, "Flow")
        self.flow_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=(0, 6), pady=(9, 0))
        self._build_flow_panel()

        self.range_frame = self._panel(cards, "Range Sector")
        self.range_frame.grid(row=1, column=2, columnspan=2, sticky="nsew", padx=(6, 0), pady=(9, 0))
        self._build_range_panel()

        console_frame = self._panel(self.root, "Status Log")
        console_frame.pack(fill="x", padx=18, pady=(0, 18))
        self.console_text = tk.Text(
            console_frame,
            height=7,
            bg=PALETTE["canvas_bg"],
            fg=PALETTE["text_main"],
            insertbackground=PALETTE["text_main"],
            relief="flat",
            highlightthickness=0,
            font=("Menlo", 10),
            state="disabled",
        )
        self.console_text.pack(fill="both", expand=True, padx=16, pady=(0, 16))

    def _build_mission_panel(self) -> None:
        body = tk.Frame(self.mission_frame, bg=PALETTE["panel_bg"])
        body.pack(fill="both", expand=True, padx=4, pady=(0, 12))

        self.outer_value = self._value_pair(body, 0, "Outer State")
        self.mission_value = self._value_pair(body, 1, "Mission State")
        self.z_value = self._value_pair(body, 2, "Height")
        self.battery_value = self._value_pair(body, 3, "Battery")
        self.max_temp_mission_value = self._value_pair(body, 4, "Max Temp")
        self.therm_mission_value = self._value_pair(body, 5, "Thermistor")
        self.hold_value = self._value_pair(body, 6, "Human Hold")
        self.conf_value = self._value_pair(body, 7, "Confidence")
        self.dir_value = self._value_pair(body, 8, "Direction")

    def _build_human_panel(self) -> None:
        body = tk.Frame(self.human_frame, bg=PALETTE["panel_bg"])
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        flags_row = tk.Frame(body, bg=PALETTE["panel_bg"])
        flags_row.pack(anchor="w", pady=(0, 10))
        self.fresh_value = tk.Label(flags_row, text="Fresh: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.fresh_value.pack(side="left")
        self.stable_value = tk.Label(flags_row, text="Stable: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.stable_value.pack(side="left", padx=(20, 0))

        self.human_led_canvas = tk.Canvas(body, width=180, height=180, bg=PALETTE["panel_bg"], highlightthickness=0)
        self.human_led_canvas.pack(pady=(6, 12))
        self.human_led_canvas.create_oval(20, 20, 160, 160, fill=PALETTE["human_off"], outline=PALETTE["text_muted"], width=4, tags="led")
        self.human_led_canvas.create_text(80, 90, text="NO\nHUMAN", fill=PALETTE["text_main"], font=("Helvetica", 18, "bold"), tags="led_text")
        self.max_temp_value = tk.Label(body, text="Max Temp: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.max_temp_value.pack(anchor="w", pady=(0, 2))
        self.therm_value = tk.Label(body, text="Thermistor: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.therm_value.pack(anchor="w")

    def _build_flow_panel(self) -> None:
        body = tk.Frame(self.flow_frame, bg=PALETTE["panel_bg"])
        body.pack(fill="both", expand=True, padx=4, pady=(0, 12))

        self.delta_value = self._value_pair(body, 0, "Delta")
        self.shutter_value = self._value_pair(body, 1, "Shutter")
        self.vel_value = self._value_pair(body, 2, "Velocity")
        self.cmd_value = self._value_pair(body, 3, "Command")
        self.pos_x_value = self._value_pair(body, 4, "X (start)")
        self.pos_y_value = self._value_pair(body, 5, "Y (start)")
        self.flow_age_value = self._value_pair(body, 6, "Timestamp")

    def _build_path_panel(self) -> None:
        body = tk.Frame(self.path_frame, bg=PALETTE["panel_bg"])
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self.path_canvas = tk.Canvas(
            body,
            width=300,
            height=420,
            bg=PALETTE["canvas_bg"],
            highlightthickness=0,
        )
        self.path_canvas.pack(expand=True)
        self.path_canvas.bind("<Configure>", lambda _event: self._render_path())

    def _build_range_panel(self) -> None:
        body = tk.Frame(self.range_frame, bg=PALETTE["panel_bg"])
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self.range_canvas = tk.Canvas(body, width=360, height=280, bg=PALETTE["canvas_bg"], highlightthickness=0)
        self.range_canvas.pack(pady=(8, 12))
        self.range_items = {
            "front_rect": self.range_canvas.create_rectangle(130, 18, 230, 88, fill=PALETTE["sector_far"], outline=PALETTE["panel_edge"], width=2),
            "front_text": self.range_canvas.create_text(180, 53, text="Front\n--", fill=PALETTE["text_main"], font=("Helvetica", 12, "bold")),
            "left_rect": self.range_canvas.create_rectangle(20, 100, 110, 180, fill=PALETTE["sector_far"], outline=PALETTE["panel_edge"], width=2),
            "left_text": self.range_canvas.create_text(65, 140, text="Left\n--", fill=PALETTE["text_main"], font=("Helvetica", 12, "bold")),
            "right_rect": self.range_canvas.create_rectangle(250, 100, 340, 180, fill=PALETTE["sector_far"], outline=PALETTE["panel_edge"], width=2),
            "right_text": self.range_canvas.create_text(295, 140, text="Right\n--", fill=PALETTE["text_main"], font=("Helvetica", 12, "bold")),
            "back_rect": self.range_canvas.create_rectangle(130, 192, 230, 262, fill=PALETTE["sector_far"], outline=PALETTE["panel_edge"], width=2),
            "back_text": self.range_canvas.create_text(180, 227, text="Back\n--", fill=PALETTE["text_main"], font=("Helvetica", 12, "bold")),
            "center": self.range_canvas.create_oval(144, 104, 216, 176, fill=PALETTE["panel_bg"], outline=PALETTE["text_muted"], width=2),
            "center_text": self.range_canvas.create_text(180, 140, text="CF", fill=PALETTE["text_main"], font=("Helvetica", 15, "bold")),
        }

        self.up_value = tk.Label(body, text="Up: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.up_value.pack(anchor="w")
        self.range_age_value = tk.Label(body, text="Timestamp: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.range_age_value.pack(anchor="w", pady=(4, 0))

    def connect(self) -> None:
        if self.ui_state == self.STATE_CONNECTED:
            self._disconnect_session("Disconnect requested")
        elif self.ui_state == self.STATE_CONNECTING:
            self._disconnect_session("Cancelling connection")
        elif self.ui_state == self.STATE_SCANNING:
            return
        else:
            self._begin_connection()

    def scan(self) -> None:
        if self.ui_state != self.STATE_DISCONNECTED:
            return

        if self.scan_worker and self.scan_worker.is_alive():
            return

        self.pending_start = False
        self.connection_var.set("Scanning")
        self.status_var.set("Scanning for nearby Crazyflies...")
        self._append_console("Scanning for nearby Crazyflies")
        self._set_ui_state(self.STATE_SCANNING)

        self.scan_worker = ScanSession(self.event_queue)
        self.scan_worker.start()

    def start_mission(self) -> None:
        self.trace_points = []
        self.trace_active = True
        self._render_path()
        if not self.worker or not self.connected:
            self.pending_start = True
            self._append_console("Start requested without an active link. Reconnecting first.")
            self._begin_connection()
            return

        self.pending_start = False
        self.worker.request_active(True)
        self._append_console("Start button pressed")

    def stop_mission(self) -> None:
        self.pending_start = False
        self.trace_active = False
        if not self.worker or not self.connected:
            return
        self.worker.request_active(False)
        self._append_console("Stop button pressed")

    def on_close(self) -> None:
        self.closing = True
        if self.worker:
            self.worker.shutdown()
        self.root.after(150, self.root.destroy)

    def _append_console(self, line: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.console_lines.append(f"[{timestamp}] {line}")
        self.console_lines = self.console_lines[-16:]
        self.console_text.configure(state="normal")
        self.console_text.delete("1.0", "end")
        self.console_text.insert("end", "\n".join(self.console_lines))
        self.console_text.see("end")
        self.console_text.configure(state="disabled")

    def _handle_status(self, payload: Dict[str, object]) -> None:
        message = str(payload.get("message", ""))
        level = str(payload.get("level", "info"))
        self.status_var.set(message)
        self._append_console(f"{level.upper()}: {message}")

    def _handle_connected(self, payload: Dict[str, object]) -> None:
        self.connected = True
        connected_uri = str(payload.get("uri", ""))
        self.uri_var.set(connected_uri)
        self.connection_var.set(f"Connected: {connected_uri}")
        self._set_ui_state(self.STATE_CONNECTED)
        self._append_console(f"Connected to {connected_uri}")
        if self.pending_start and self.worker:
            self.pending_start = False
            self.status_var.set("Link restored. Sending app.active=1")
            self.worker.request_active(True)
            self._append_console("Auto-starting mission after reconnect")

    def _handle_disconnected(self) -> None:
        self.connected = False
        self.trace_active = False
        self.connection_var.set("Disconnected")
        self.worker = None
        self._set_ui_state(self.STATE_DISCONNECTED)
        if not self.closing:
            self.status_var.set("Disconnected")
            self._append_console("Radio session stopped")

    def _append_trace_point(self) -> None:
        if not self.trace_active or not self.flow.pos_valid:
            return

        point = (self.flow.pos_x_cm, self.flow.pos_y_cm, self.flow.pos_z_cm)
        if self.trace_points:
            last_x, last_y, last_z = self.trace_points[-1]
            dist_sq = ((point[0] - last_x) ** 2) + ((point[1] - last_y) ** 2) + ((point[2] - last_z) ** 2)
            if dist_sq < 0.25:
                return

        self.trace_points.append(point)
        if len(self.trace_points) > 1600:
            self.trace_points = self.trace_points[-1600:]

    def _render_path(self) -> None:
        if not hasattr(self, "path_canvas"):
            return

        canvas = self.path_canvas
        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)
        canvas.delete("all")

        margin = 24.0
        if not self.trace_points:
            canvas.create_text(
                width / 2,
                height / 2,
                text="Trace starts after Start Mission",
                fill=PALETTE["text_dim"],
                font=("Helvetica", 14, "bold"),
            )
            return

        samples = [(0.0, 0.0, 0.0)] + self.trace_points

        def project(point: tuple[float, float, float]) -> tuple[float, float]:
            x_cm, y_cm, z_cm = point
            return (x_cm + 0.55 * y_cm, -0.30 * y_cm - 0.85 * z_cm)

        def project_shadow(point: tuple[float, float, float]) -> tuple[float, float]:
            x_cm, y_cm, _z_cm = point
            return (x_cm + 0.55 * y_cm, -0.30 * y_cm)

        projected = [project(point) for point in samples]
        shadows = [project_shadow(point) for point in samples]
        combined = projected + shadows

        min_u = min(u for u, _ in combined)
        max_u = max(u for u, _ in combined)
        min_v = min(v for _, v in combined)
        max_v = max(v for _, v in combined)

        span_u = max(max_u - min_u, 50.0)
        span_v = max(max_v - min_v, 40.0)
        scale = min((width - 2.0 * margin) / span_u, (height - 2.0 * margin) / span_v)
        scale = max(1.1, min(scale, 4.2))

        offset_x = (width - span_u * scale) / 2.0 - min_u * scale
        offset_y = (height - span_v * scale) / 2.0 - min_v * scale

        def to_screen(proj: tuple[float, float]) -> tuple[float, float]:
            u, v = proj
            return (offset_x + u * scale, offset_y + v * scale)

        shadow_points = [to_screen(proj) for proj in shadows]
        path_points = [to_screen(proj) for proj in projected]

        floor_polygon = [
            to_screen(project_shadow((-35.0, 0.0, 0.0))),
            to_screen(project_shadow((35.0, 0.0, 0.0))),
            to_screen(project_shadow((35.0, 55.0, 0.0))),
            to_screen(project_shadow((-35.0, 55.0, 0.0))),
        ]
        canvas.create_polygon(
            *[coord for point in floor_polygon for coord in point],
            fill="#101010",
            outline="#1b1b1b",
            width=1,
        )

        start_shadow = shadow_points[0]
        x_axis = to_screen(project_shadow((18.0, 0.0, 0.0)))
        y_axis = to_screen(project_shadow((0.0, 18.0, 0.0)))
        z_axis = to_screen(project((0.0, 0.0, 18.0)))
        canvas.create_line(*start_shadow, *x_axis, fill="#2f2f2f", width=2)
        canvas.create_line(*start_shadow, *y_axis, fill="#2f2f2f", width=2)
        canvas.create_line(*start_shadow, *z_axis, fill="#383838", width=2)
        canvas.create_text(start_shadow[0] + 8, start_shadow[1] + 12, text="START", fill=PALETTE["text_dim"], font=("Helvetica", 10, "bold"), anchor="w")

        shadow_flat = [coord for point in shadow_points for coord in point]
        path_flat = [coord for point in path_points for coord in point]

        if len(shadow_points) >= 2:
            canvas.create_line(*shadow_flat, fill="#3a2417", width=7, smooth=True, splinesteps=12)

        step = max(1, len(path_points) // 10)
        for idx in range(step, len(path_points), step):
            canvas.create_line(
                *shadow_points[idx],
                *path_points[idx],
                fill="#4a382d",
                width=1,
            )

        if len(path_points) >= 2:
            canvas.create_line(*path_flat, fill="#FC4C02", width=4, smooth=True, splinesteps=12)

        head_x, head_y = path_points[-1]
        shadow_x, shadow_y = shadow_points[-1]
        canvas.create_line(shadow_x, shadow_y, head_x, head_y, fill="#6d4b3a", width=2)

        breath = 0.5 * (1.0 + math.sin(self.path_breath_phase))
        glow_r = 10.0 + 4.0 * breath
        mid_r = 6.5 + 2.0 * breath
        head_r = 4.0 + 1.0 * breath

        canvas.create_oval(head_x - glow_r, head_y - glow_r, head_x + glow_r, head_y + glow_r, outline="#ff8a75", width=2)
        canvas.create_oval(head_x - mid_r, head_y - mid_r, head_x + mid_r, head_y + mid_r, outline="#ff5a4f", width=3)
        canvas.create_oval(head_x - head_r, head_y - head_r, head_x + head_r, head_y + head_r, fill="#ff3b30", outline="#ffd4ce", width=1)

    def _animate_path_head(self) -> None:
        self.path_breath_phase += 0.35
        if self.trace_points:
            self._render_path()
        self.root.after(80, self._animate_path_head)

    def _handle_scan_result(self, payload: Dict[str, object]) -> None:
        interfaces = [str(uri) for uri in payload.get("interfaces", [])]
        selected_uri = str(payload.get("selected_uri") or "")
        self.scan_worker = None
        self.last_scan_uris = interfaces

        if interfaces:
            if selected_uri:
                self.uri_var.set(selected_uri)
            self.connection_var.set(f"Found {len(interfaces)} interface(s)")
            if selected_uri:
                self.status_var.set(f"Scan complete. Selected {selected_uri}")
            else:
                self.status_var.set("Scan complete")
            self._append_console(f"Scan found: {', '.join(interfaces)}")
        else:
            self.connection_var.set("No interfaces found")
            self.status_var.set("No Crazyflie interfaces found")
            self._append_console("No Crazyflie interfaces found")

        self._set_ui_state(self.STATE_DISCONNECTED)

    def _set_uri_entry_state(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self.uri_entry.configure(
            state=state,
            disabledbackground=PALETTE["field_bg"],
            disabledforeground=PALETTE["text_dim"],
        )

    def _set_ui_state(self, state: str) -> None:
        self.ui_state = state

        if state == self.STATE_DISCONNECTED:
            self.connect_button.configure(text="Connect")
            self.connect_button.set_state("normal")
            self.scan_button.configure(text="Scan")
            self.scan_button.set_state("normal")
            self.start_button.set_state("disabled")
            self.stop_button.set_state("disabled")
            self._set_uri_entry_state(True)
        elif state == self.STATE_CONNECTING:
            self.connect_button.configure(text="Cancel")
            self.connect_button.set_state("normal")
            self.scan_button.configure(text="Scan")
            self.scan_button.set_state("disabled")
            self.start_button.set_state("disabled")
            self.stop_button.set_state("disabled")
            self._set_uri_entry_state(False)
        elif state == self.STATE_CONNECTED:
            self.connect_button.configure(text="Disconnect")
            self.connect_button.set_state("normal")
            self.scan_button.configure(text="Scan")
            self.scan_button.set_state("disabled")
            self.start_button.set_state("normal")
            self.stop_button.set_state("normal")
            self._set_uri_entry_state(False)
        elif state == self.STATE_SCANNING:
            self.connect_button.configure(text="Connect")
            self.connect_button.set_state("disabled")
            self.scan_button.configure(text="Scanning...")
            self.scan_button.set_state("disabled")
            self.start_button.set_state("disabled")
            self.stop_button.set_state("disabled")
            self._set_uri_entry_state(False)

    def _disconnect_session(self, reason: str) -> None:
        self.pending_start = False

        if self.worker and self.worker.is_alive():
            self.connection_var.set("Disconnecting")
            self.status_var.set(reason)
            self._append_console(reason)
            self.worker.shutdown()
            self.worker.join(timeout=2.0)
            if self.worker.is_alive():
                self.status_var.set("Previous radio session is still shutting down")
                self._append_console("ERROR: Could not stop the previous radio session cleanly")
                return

        self.worker = None
        self._handle_disconnected()

    def _begin_connection(self) -> None:
        if self.worker and self.worker.is_alive():
            self.worker.shutdown()
            self.worker.join(timeout=2.0)
            if self.worker.is_alive():
                self.status_var.set("Previous radio session is still shutting down")
                self._append_console("ERROR: Could not restart connection because the previous session is still alive")
                return

        self.connected = False
        self.worker = None
        preferred_uri = self.uri_var.get().strip() or None
        if preferred_uri:
            self.connection_var.set("Connecting")
            self.status_var.set(f"Connecting to {preferred_uri}...")
            self._append_console(f"Connecting to {preferred_uri}")
        else:
            self.connection_var.set("Scanning")
            self.status_var.set("Scanning for nearby Crazyflies...")
            self._append_console("Scanning for nearby Crazyflies")
        self._set_ui_state(self.STATE_CONNECTING)

        self.worker = RadioSession(
            preferred_uri=preferred_uri,
            cache_dir=self.args.cache,
            csv_dir=self.args.csv_dir,
            log_period_ms=self.args.log_period_ms,
            event_queue=self.event_queue,
        )
        self.worker.start()

    def _pump_events(self) -> None:
        while True:
            try:
                event_type, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "status":
                self._handle_status(payload)
            elif event_type == "connected":
                self._handle_connected(payload)
            elif event_type == "disconnected":
                self._handle_disconnected()
            elif event_type == "scan_result":
                self._handle_scan_result(payload)
            elif event_type == "mission":
                self.mission = MissionSnapshot(**payload)
                self._render_mission()
                self._render_human()
            elif event_type == "flow":
                self.flow = FlowSnapshot(**payload)
                self._append_trace_point()
                self._render_flow()
                self._render_path()
            elif event_type == "range":
                self.range = RangeSnapshot(**payload)
                self._render_mission()
                self._render_range()
            elif event_type == "console":
                self._append_console(f"CF: {payload.get('line', '')}")

        self.root.after(60, self._pump_events)

    def _render_all(self) -> None:
        self._render_mission()
        self._render_human()
        self._render_flow()
        self._render_path()
        self._render_range()

    def _render_mission(self) -> None:
        self.outer_value.configure(text=OUTER_NAMES.get(self.mission.outer, str(self.mission.outer)))
        self.mission_value.configure(text=MISSION_NAMES.get(self.mission.mission, str(self.mission.mission)))
        self.z_value.configure(text=f"{self.mission.z:.2f} m")
        self.battery_value.configure(text=f"{self.range.vbat:.2f} V" if self.range.vbat else "--")
        self.max_temp_mission_value.configure(text=f"{self.mission.max_temp_c:.2f} C")
        self.therm_mission_value.configure(text=f"{self.mission.therm_c:.2f} C")
        self.hold_value.configure(text=f"{self.mission.hold:.2f} s")
        self.conf_value.configure(text=f"{self.mission.conf} %")
        self.dir_value.configure(text=DIR_NAMES.get(self.mission.direction, str(self.mission.direction)))

    def _render_human(self) -> None:
        led_color = human_led_color(self.mission)
        human_text = "HUMAN" if self.mission.human else "NO\nHUMAN"
        if self.mission.human and self.mission.stable:
            human_text = "STABLE"
        elif self.mission.human and self.mission.fresh:
            human_text = "FRESH"

        self.human_led_canvas.itemconfigure("led", fill=led_color)
        self.human_led_canvas.itemconfigure("led_text", text=human_text)
        self.fresh_value.configure(text=f"Fresh: {self.mission.fresh}")
        self.stable_value.configure(text=f"Stable: {self.mission.stable}")
        self.max_temp_value.configure(text=f"Max Temp: {self.mission.max_temp_c:.2f} C")
        self.therm_value.configure(text=f"Thermistor: {self.mission.therm_c:.2f} C")

    def _render_flow(self) -> None:
        self.delta_value.configure(text=f"({self.flow.delta_x}, {self.flow.delta_y})")
        self.shutter_value.configure(text=str(self.flow.shutter))
        self.vel_value.configure(text=f"({self.flow.vel_x:.2f}, {self.flow.vel_y:.2f}) m/s")
        self.cmd_value.configure(text=f"({self.flow.cmd_x:.2f}, {self.flow.cmd_y:.2f}) m/s")
        if self.flow.pos_valid:
            self.pos_x_value.configure(text=f"{self.flow.pos_x_cm:.1f} cm")
            self.pos_y_value.configure(text=f"{self.flow.pos_y_cm:.1f} cm")
        else:
            self.pos_x_value.configure(text="--")
            self.pos_y_value.configure(text="--")
        self.flow_age_value.configure(text=f"{self.flow.timestamp_ms} ms")

    def _render_range(self) -> None:
        sectors = {
            "front": self.range.front,
            "left": self.range.left,
            "right": self.range.right,
            "back": self.range.back,
        }
        for direction, distance in sectors.items():
            self.range_canvas.itemconfigure(self.range_items[f"{direction}_rect"], fill=range_fill(distance))
            self.range_canvas.itemconfigure(
                self.range_items[f"{direction}_text"],
                text=f"{direction.title()}\n{pretty_distance(distance)}",
            )

        self.up_value.configure(text=f"Up: {pretty_distance(self.range.up)}")
        self.range_age_value.configure(text=f"Timestamp: {self.range.timestamp_ms} ms")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tk dashboard for the Crazyflie wall-follow mission")
    parser.add_argument("--uri", help="Crazyflie link URI. Defaults to usb://0.")
    parser.add_argument("--cache", default="/Users/zhangzehua/Desktop/fyp/cache", help="TOC cache directory")
    parser.add_argument(
        "--csv-dir",
        default=str(SCRIPT_DIR / "telemetry_logs"),
        help="Directory where realtime telemetry CSV files will be written",
    )
    parser.add_argument("--log-period-ms", type=int, default=250, help="Telemetry log period in ms")
    parser.add_argument("--no-auto-connect", dest="auto_connect", action="store_false", help="Open the window without connecting immediately")
    parser.set_defaults(auto_connect=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = tk.Tk()
    DashboardApp(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
