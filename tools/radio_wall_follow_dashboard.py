#!/usr/bin/env python3
"""Tk dashboard for the Crazyflie wall-follow mission over Crazyradio."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import queue
import shutil
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

try:
    from PIL import Image, ImageDraw
except ImportError:
    Image = None
    ImageDraw = None

PLOTLY_IMPORT_ERROR: Optional[str] = None

try:
    import plotly.graph_objects as go
except Exception as exc:
    go = None
    PLOTLY_IMPORT_ERROR = str(exc)

try:
    from playsound import playsound as playsound_fn
except ImportError:
    playsound_fn = None

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
    3: "Approach",
    4: "Bob",
    5: "Land",
    6: "Transition",
    7: "Align",
}

DIR_NAMES = {
    -1: "Left",
    0: "Center",
    1: "Right",
}


def ensure_plotly():
    global go
    global PLOTLY_IMPORT_ERROR

    if go is not None:
        return go

    try:
        import plotly.graph_objects as plotly_go
    except Exception as exc:
        PLOTLY_IMPORT_ERROR = str(exc)
        return None

    go = plotly_go
    PLOTLY_IMPORT_ERROR = None
    return go

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

        timestamp = datetime.now().strftime("%d_%H_%M_%S")
        base_dir = pathlib.Path(csv_dir).expanduser()
        base_dir.mkdir(parents=True, exist_ok=True)
        self.path = base_dir / f"Log_{timestamp}.csv"
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


class SoundEffectPlayer:
    def __init__(self) -> None:
        self._queue: "queue.Queue[Optional[pathlib.Path]]" = queue.Queue()
        self._afplay_path = shutil.which("afplay")
        self._worker = threading.Thread(target=self._run, name="dashboard-sfx", daemon=True)
        self._worker.start()

    def available(self) -> bool:
        return self._afplay_path is not None or playsound_fn is not None

    def play(self, sound_path: pathlib.Path) -> bool:
        if not sound_path.exists() or not self.available():
            return False
        self._queue.put(sound_path)
        return True

    def shutdown(self) -> None:
        self._queue.put(None)

    def _run(self) -> None:
        while True:
            sound_path = self._queue.get()
            if sound_path is None:
                break

            try:
                if self._afplay_path is not None:
                    process = subprocess.Popen(
                        [self._afplay_path, str(sound_path)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    process.wait()
                elif playsound_fn is not None:
                    playsound_fn(str(sound_path))
            except Exception:
                pass


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
        self.command_queue: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.cf: Optional[Crazyflie] = None

    def request_active(self, active: bool) -> None:
        self.command_queue.put(("set_active", 1 if active else 0))

    def request_capture_start(self) -> None:
        self.command_queue.put(("capture_start", None))

    def request_capture_stop_after(self, delay_s: float) -> None:
        self.command_queue.put(("capture_stop_after", delay_s))

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
        capture_active = False
        capture_stop_deadline: Optional[float] = None

        def capture_enabled_now() -> bool:
            nonlocal capture_active
            nonlocal capture_stop_deadline

            if capture_active and capture_stop_deadline is not None and time.monotonic() >= capture_stop_deadline:
                capture_active = False
                capture_stop_deadline = None

            return capture_active
        origin_world_x: Optional[float] = None
        origin_world_y: Optional[float] = None
        origin_world_z: Optional[float] = None
        origin_yaw_deg: Optional[float] = None
        mission_outer_var: Optional[str] = None
        mission_state_var: Optional[str] = None
        mission_human_var: Optional[str] = None
        mission_conf_var: Optional[str] = None
        mission_dir_var: Optional[str] = None
        mission_fresh_var: Optional[str] = None
        mission_stable_var: Optional[str] = None
        mission_hold_var: Optional[str] = None
        mission_max_var: Optional[str] = None
        mission_therm_var: Optional[str] = None
        mission_z_var: Optional[str] = None

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

        def log_var_exists(complete_name: str) -> bool:
            if "." not in complete_name:
                return False
            if cf.log.toc is None:
                return False
            group, name = complete_name.split(".", 1)
            return cf.log.toc.get_element(group, name) is not None

        def choose_log_var(*candidates: str) -> Optional[str]:
            for candidate in candidates:
                if log_var_exists(candidate):
                    return candidate
            return None

        def on_mission(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            snapshot = MissionSnapshot(
                timestamp_ms=ts,
                outer=int(data.get(mission_outer_var, 0)) if mission_outer_var else 0,
                mission=int(data.get(mission_state_var, 0)) if mission_state_var else 0,
                human=int(data.get(mission_human_var, 0)) if mission_human_var else 0,
                fresh=int(data.get(mission_fresh_var, 0)) if mission_fresh_var else 0,
                stable=int(data.get(mission_stable_var, 0)) if mission_stable_var else 0,
                hold=float(data.get(mission_hold_var, 0.0)) if mission_hold_var else 0.0,
                conf=int(data.get(mission_conf_var, 0)) if mission_conf_var else 0,
                direction=int(data.get(mission_dir_var, 0)) if mission_dir_var else 0,
                z=float(data.get(mission_z_var, 0.0)) if mission_z_var else 0.0,
                max_temp_c=(int(data.get(mission_max_var, 0)) / 100.0) if mission_max_var else 0.0,
                therm_c=(int(data.get(mission_therm_var, 0)) / 100.0) if mission_therm_var else 0.0,
            )
            if csv_logger is not None and capture_enabled_now():
                csv_logger.log_mission(snapshot)
            self._emit("mission", **snapshot.__dict__)

        def emit_flow_snapshot() -> None:
            nonlocal latest_flow
            snapshot = FlowSnapshot(**latest_flow.__dict__)
            if csv_logger is not None and capture_enabled_now():
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
            if csv_logger is not None and capture_enabled_now():
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
                mission_outer_var = choose_log_var("app.stateOuterLoop", "app.stateOuter")
                mission_state_var = choose_log_var("app.missionState", "app.mission")
                mission_human_var = choose_log_var("app.humanDetected", "app.human")
                mission_conf_var = choose_log_var("app.humanConf")
                mission_dir_var = choose_log_var("app.humanDir")
                mission_fresh_var = choose_log_var("app.humanFresh")
                mission_stable_var = choose_log_var("app.humanStable")
                mission_hold_var = choose_log_var("app.humanHold")
                mission_max_var = choose_log_var("app.humanMax")
                mission_therm_var = choose_log_var("app.humanTherm")
                mission_z_var = choose_log_var("stateEstimate.z")
                try:
                    csv_logger = TelemetryCsvLogger(self.csv_dir, uri)
                    self._emit("status", level="info", message=f"CSV telemetry logging to {csv_logger.path}")
                except Exception as exc:
                    self._emit("status", level="error", message=f"Failed to start CSV telemetry logging: {exc}")

                active_logconfs: list[LogConfig] = []

                mission_cfg = LogConfig(name="mission", period_in_ms=self.log_period_ms)
                mission_log_vars = [
                    (mission_outer_var, "uint8_t"),
                    (mission_state_var, "uint8_t"),
                    (mission_human_var, "uint8_t"),
                    (mission_conf_var, "uint8_t"),
                    (mission_dir_var, "int8_t"),
                    (mission_fresh_var, "uint8_t"),
                    (mission_stable_var, "uint8_t"),
                    (mission_hold_var, "float"),
                    (mission_max_var, "int16_t"),
                    (mission_therm_var, "int16_t"),
                    (mission_z_var, "float"),
                ]
                mission_var_names = [name for name, _ctype in mission_log_vars if name]
                for var_name, ctype in mission_log_vars:
                    if var_name:
                        mission_cfg.add_variable(var_name, ctype)
                if mission_var_names:
                    mission_cfg.data_received_cb.add_callback(on_mission)
                    mission_cfg.error_cb.add_callback(on_log_error)
                    try:
                        cf.log.add_config(mission_cfg)
                    except Exception as exc:
                        self._emit("status", level="warning", message=f"Skipping mission log block: {exc}")
                    else:
                        active_logconfs.append(mission_cfg)
                        self._emit(
                            "status",
                            level="info",
                            message=f"Mission log TOC matched: {', '.join(mission_var_names)}",
                        )
                else:
                    self._emit("status", level="warning", message="No compatible mission log variables found in TOC; mission panel will stay default")

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
                try:
                    cf.log.add_config(flow_cfg)
                except Exception as exc:
                    self._emit("status", level="warning", message=f"Skipping flow log block: {exc}")
                else:
                    active_logconfs.append(flow_cfg)

                pose_cfg = LogConfig(name="pose", period_in_ms=self.log_period_ms)
                pose_cfg.add_variable("stateEstimate.x", "float")
                pose_cfg.add_variable("stateEstimate.y", "float")
                pose_cfg.add_variable("stateEstimate.z", "float")
                pose_cfg.add_variable("stabilizer.yaw", "float")
                pose_cfg.data_received_cb.add_callback(on_pose)
                pose_cfg.error_cb.add_callback(on_log_error)
                try:
                    cf.log.add_config(pose_cfg)
                except Exception as exc:
                    self._emit("status", level="warning", message=f"Skipping pose log block: {exc}")
                else:
                    active_logconfs.append(pose_cfg)

                range_cfg = LogConfig(name="range", period_in_ms=self.log_period_ms)
                range_cfg.add_variable("range.front", "uint16_t")
                range_cfg.add_variable("range.left", "uint16_t")
                range_cfg.add_variable("range.right", "uint16_t")
                range_cfg.add_variable("range.back", "uint16_t")
                range_cfg.add_variable("range.up", "uint16_t")
                range_cfg.add_variable("pm.vbat", "float")
                range_cfg.data_received_cb.add_callback(on_range)
                range_cfg.error_cb.add_callback(on_log_error)
                try:
                    cf.log.add_config(range_cfg)
                except Exception as exc:
                    self._emit("status", level="warning", message=f"Skipping range log block: {exc}")
                else:
                    active_logconfs.append(range_cfg)

                logconfs.extend(active_logconfs)
                for cfg in logconfs:
                    try:
                        cfg.start()
                    except Exception as exc:
                        self._emit("status", level="warning", message=f"Failed to start {cfg.name} log block: {exc}")

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
                    elif command == "capture_start":
                        capture_active = True
                        capture_stop_deadline = None
                        self._emit("status", level="info", message="Mission telemetry capture started")
                    elif command == "capture_stop_after":
                        if capture_active:
                            capture_stop_deadline = time.monotonic() + max(float(value or 0.0), 0.0)
                            self._emit("status", level="info", message=f"Mission telemetry capture will stop in {float(value or 0.0):.1f}s")
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
        self.show_trace_position = False
        self.path_breath_phase = 0.0
        self.sfx_player = SoundEffectPlayer()
        self.beep_sound_path = SCRIPT_DIR / "sound" / "beep.wav"
        self.human_confirmed_sound_path = SCRIPT_DIR / "sound" / "human_confirmed.wav"
        self.last_human_sound_snapshot = MissionSnapshot()
        self.sound_warning_shown = False
        self.trace_output_dir = pathlib.Path(args.csv_dir).expanduser()
        self.trace_session_token: Optional[str] = None
        self.trace_session_uri: Optional[str] = None
        self.trace_exported = False
        self.stop_capture_delay_ms: int = 3000  # Time after mission stop to keep capturing telemetry for trace export.
        self.trace_stop_after_id: Optional[str] = None
        self.latest_trace_html: Optional[pathlib.Path] = None
        self.console_lines: list[str] = []
        initial_uri = args.uri.strip() if args.uri else ""
        self.uri_var = tk.StringVar(value=initial_uri)
        self.status_var = tk.StringVar(value="Press Scan to discover a Crazyflie or enter a URI")
        self.connection_var = tk.StringVar(value="Disconnected")

        self._configure_root()
        self._build_layout()
        self._render_all()
        self._set_ui_state(self.STATE_DISCONNECTED)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(60, self._pump_events)
        self.root.after(80, self._animate_path_head)
        if self.args.auto_connect and initial_uri:
            self.root.after(150, self.connect)

    def _configure_root(self) -> None:
        self.root.title("Crazyflie Mission Dashboard")
        self.root.geometry("1280x960")
        self.root.minsize(1160, 860)
        self.root.configure(bg=PALETTE["app_bg"])

    def _panel(self, parent: tk.Widget, title: str) -> tk.Frame:
        frame = tk.Frame(parent, bg=PALETTE["panel_bg"], bd=0, highlightthickness=1, highlightbackground=PALETTE["panel_edge"])
        header_row = tk.Frame(frame, bg=PALETTE["panel_bg"])
        header_row.pack(fill="x", padx=16, pady=(14, 8))
        header = tk.Label(
            header_row,
            text=title,
            bg=PALETTE["panel_bg"],
            fg=PALETTE["text_main"],
            font=("Helvetica", 14, "bold"),
            anchor="w",
        )
        header.pack(side="left", fill="x", expand=True)
        setattr(frame, "_header_row", header_row)
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
        self.open_path_button = ColorButton(
            getattr(self.path_frame, "_header_row"),
            text="Open 3D",
            command=self.open_trace_html,
            bg="#252525",
            fg="#ffffff",
            active_bg="#1b1b1b",
            active_fg="#ffffff",
            width=9,
            font=("Helvetica", 10, "bold"),
        )
        self.open_path_button.set_state("disabled")
        self.open_path_button.pack(side="right")
        self.stop_trace_button = ColorButton(
            getattr(self.path_frame, "_header_row"),
            text="Stop",
            command=self.stop_trace_capture,
            bg="#252525",
            fg="#ffffff",
            active_bg="#1b1b1b",
            active_fg="#ffffff",
            width=8,
            font=("Helvetica", 10, "bold"),
        )
        self.stop_trace_button.pack(side="right", padx=(0, 8))
        self.clear_path_button = ColorButton(
            getattr(self.path_frame, "_header_row"),
            text="Clear",
            command=self.clear_trace_dashboard,
            bg="#252525",
            fg="#ffffff",
            active_bg="#1b1b1b",
            active_fg="#ffffff",
            width=8,
            font=("Helvetica", 10, "bold"),
        )
        self.clear_path_button.pack(side="right", padx=(0, 8))
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
        flags_row.pack(fill="x", pady=(0, 10))
        self.fresh_value = tk.Label(flags_row, text="Fresh: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.fresh_value.pack(side="left")
        self.stable_value = tk.Label(flags_row, text="Stable: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.stable_value.pack(side="left", padx=(20, 0))
        self.uart_status_label = tk.Label(flags_row, text="UART", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11, "bold"))
        self.uart_status_label.pack(side="right", padx=(8, 0))
        self.uart_status_canvas = tk.Canvas(flags_row, width=16, height=16, bg=PALETTE["panel_bg"], highlightthickness=0)
        self.uart_status_canvas.pack(side="right")
        self.uart_status_canvas.create_rectangle(2, 2, 14, 14, fill="#d93025", outline=PALETTE["text_muted"], width=1, tags="uart_led")

        self.human_led_canvas = tk.Canvas(body, width=180, height=180, bg=PALETTE["panel_bg"], highlightthickness=0)
        self.human_led_canvas.pack(pady=(6, 12))
        self.human_led_canvas.create_oval(20, 20, 160, 160, fill=PALETTE["human_off"], outline=PALETTE["text_muted"], width=4, tags="led")
        self.human_led_canvas.create_text(90, 90, text="NO\nHUMAN", fill=PALETTE["text_main"], font=("Helvetica", 18, "bold"), tags="led_text")
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
            width=560,
            height=300,
            bg=PALETTE["canvas_bg"],
            highlightthickness=0,
        )
        self.path_canvas.pack(fill="both", expand=True)
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
        self._cancel_trace_stop_timer()
        self._export_trace_image_if_needed()
        self.trace_points = []
        self.trace_active = True
        self.show_trace_position = True
        self.trace_session_token = datetime.now().strftime("%d_%H_%M_%S")
        self.trace_session_uri = self.uri_var.get().strip() or "unknown_uri"
        self.trace_exported = False
        self._render_path()
        if not self.worker or not self.connected:
            self.pending_start = True
            self._append_console("Start requested without an active link. Reconnecting first.")
            self._begin_connection()
            return

        self.pending_start = False
        self.worker.request_capture_start()
        self.worker.request_active(True)
        self._append_console("Start button pressed")

    def stop_mission(self) -> None:
        self.pending_start = False
        if not self.worker or not self.connected:
            self._finish_trace_capture_window()
            return
        self._cancel_trace_stop_timer()
        if self.trace_active:
            self.trace_stop_after_id = self.root.after(int(self.stop_capture_delay_ms), self._finish_trace_capture_window)
        self.worker.request_capture_stop_after(self.stop_capture_delay_ms / 1000.0)
        self.worker.request_active(False)
        self._append_console("Stop button pressed; capture will stop after landing window")

    def on_close(self) -> None:
        self.closing = True
        self._cancel_trace_stop_timer()
        self.trace_active = False
        self._export_trace_image_if_needed()
        self.sfx_player.shutdown()
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

    def open_trace_html(self) -> None:
        if not self.trace_points:
            self._append_console("No trace points have been recorded for the current session yet")
            return

        if self.trace_points and not self.trace_exported:
            self._export_trace_image_if_needed()

        if self.latest_trace_html is not None and self.latest_trace_html.exists():
            webbrowser.open(self.latest_trace_html.as_uri())
            self._append_console(f"Opened 3D path in browser: {self.latest_trace_html}")
            return

        self._append_console("No exported 3D path is available yet")

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
            self.worker.request_capture_start()
            self.worker.request_active(True)
            self._append_console("Auto-starting mission after reconnect")

    def _handle_disconnected(self) -> None:
        self.connected = False
        self.last_human_sound_snapshot = MissionSnapshot()
        self._cancel_trace_stop_timer()
        self.trace_active = False
        self.show_trace_position = False
        self._export_trace_image_if_needed()
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
        if len(self.trace_points) > 2500:
            self.trace_points = self.trace_points[-2500:]

    def _cancel_trace_stop_timer(self) -> None:
        if self.trace_stop_after_id is None:
            return
        try:
            self.root.after_cancel(self.trace_stop_after_id)
        except Exception:
            pass
        self.trace_stop_after_id = None

    def _finish_trace_capture_window(self) -> None:
        self.trace_stop_after_id = None
        if not self.trace_active:
            return
        self.trace_active = False
        self._export_trace_image_if_needed()
        self._append_console("Telemetry capture window closed")

    def stop_trace_capture(self) -> None:
        self._cancel_trace_stop_timer()
        if self.worker and self.connected:
            self.worker.request_capture_stop_after(0.0)

        if not self.trace_active:
            self._append_console("Trace capture is not active")
            return

        self.trace_active = False
        self._export_trace_image_if_needed()
        self._append_console("Trace capture stopped immediately")

    def clear_trace_dashboard(self) -> None:
        if self.trace_active or self.trace_stop_after_id is not None:
            self._append_console("Cannot clear the flying path while a trace session is still active")
            return

        self.trace_points = []
        self.show_trace_position = False
        self.trace_exported = False
        self.trace_session_token = None
        self.trace_session_uri = None
        self.latest_trace_html = None
        self.open_path_button.set_state("disabled")
        self.flow.pos_valid = 0
        self.flow.pos_x_cm = 0.0
        self.flow.pos_y_cm = 0.0
        self.flow.pos_z_cm = 0.0
        self._render_flow()
        self._render_path()
        self._append_console("Cleared flying path trace and mission-relative XY display")

    def _unique_export_path(self, output_path: pathlib.Path) -> pathlib.Path:
        if not output_path.exists():
            return output_path

        stem = output_path.stem
        suffix = output_path.suffix
        parent = output_path.parent
        counter = 1
        while True:
            candidate = parent / f"{stem}_{counter}{suffix}"
            if not candidate.exists():
                return candidate
            counter += 1

    def _play_human_sfx(self, previous: MissionSnapshot, current: MissionSnapshot) -> None:
        if not self.connected:
            return

        if not self.sfx_player.available():
            if not self.sound_warning_shown:
                self.sound_warning_shown = True
                self._append_console("WARNING: No audio playback backend is available for dashboard sound effects")
            return

        if not self.beep_sound_path.exists() or not self.human_confirmed_sound_path.exists():
            if not self.sound_warning_shown:
                self.sound_warning_shown = True
                self._append_console("WARNING: Human-detection sound files are missing in tools/sound")
            return

        if not previous.human and current.human:
            self.sfx_player.play(self.beep_sound_path)

        if not previous.stable and current.stable:
            self.sfx_player.play(self.human_confirmed_sound_path)

    def _build_path_scene(self, width: float, height: float) -> Optional[Dict[str, object]]:
        if not self.trace_points:
            return None

        margin = 24.0
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
        start_shadow = shadow_points[0]

        floor_polygon = [
            to_screen(project_shadow((-35.0, 0.0, 0.0))),
            to_screen(project_shadow((35.0, 0.0, 0.0))),
            to_screen(project_shadow((35.0, 55.0, 0.0))),
            to_screen(project_shadow((-35.0, 55.0, 0.0))),
        ]

        x_axis = to_screen(project_shadow((18.0, 0.0, 0.0)))
        y_axis = to_screen(project_shadow((0.0, 18.0, 0.0)))
        z_axis = to_screen(project((0.0, 0.0, 18.0)))
        x_axis_label = to_screen(project_shadow((22.0, 0.0, 0.0)))
        y_axis_label = to_screen(project_shadow((0.0, 22.0, 0.0)))

        return {
            "floor_polygon": floor_polygon,
            "start_shadow": start_shadow,
            "x_axis": x_axis,
            "y_axis": y_axis,
            "z_axis": z_axis,
            "x_axis_label": x_axis_label,
            "y_axis_label": y_axis_label,
            "shadow_points": shadow_points,
            "path_points": path_points,
        }

    def _render_path(self) -> None:
        if not hasattr(self, "path_canvas"):
            return

        canvas = self.path_canvas
        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)
        canvas.delete("all")

        scene = self._build_path_scene(width, height)
        if scene is None:
            canvas.create_text(
                width / 2,
                height / 2,
                text="Trace starts after Start Mission",
                fill=PALETTE["text_dim"],
                font=("Helvetica", 14, "bold"),
            )
            return

        canvas.create_polygon(
            *[coord for point in scene["floor_polygon"] for coord in point],
            fill="#101010",
            outline="#1b1b1b",
            width=1,
        )

        start_shadow = scene["start_shadow"]
        canvas.create_line(*start_shadow, *scene["x_axis"], fill="#3b3b3b", width=2)
        canvas.create_line(*start_shadow, *scene["y_axis"], fill="#3b3b3b", width=2)
        canvas.create_line(*start_shadow, *scene["z_axis"], fill="#383838", width=2)
        canvas.create_text(start_shadow[0] + 8, start_shadow[1] + 12, text="START", fill=PALETTE["text_dim"], font=("Helvetica", 10, "bold"), anchor="w")
        canvas.create_text(scene["x_axis_label"][0] + 8, scene["x_axis_label"][1] + 2, text="X", fill="#7a7a7a", font=("Helvetica", 11, "bold"), anchor="w")
        canvas.create_text(scene["y_axis_label"][0] + 8, scene["y_axis_label"][1] + 2, text="Y", fill="#7a7a7a", font=("Helvetica", 11, "bold"), anchor="w")

        shadow_points = scene["shadow_points"]
        path_points = scene["path_points"]
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

    def _export_trace_image_if_needed(self) -> None:
        if self.trace_exported:
            return

        if not self.trace_points:
            if self.trace_session_token is not None:
                self._append_console("No trace points were recorded; skipping path export")
            return

        timestamp = self.trace_session_token or datetime.now().strftime("%d_%H_%M_%S")
        self.trace_output_dir.mkdir(parents=True, exist_ok=True)
        image_path = self._unique_export_path(self.trace_output_dir / f"PathOverview_{timestamp}_Log.jpg")
        html_path = self._unique_export_path(self.trace_output_dir / f"3DPlot_{timestamp}.html")
        image_saved = self._save_path_image(image_path)
        html_saved = self._save_path_html(html_path)
        self.trace_exported = image_saved and html_saved
        if html_saved:
            self.latest_trace_html = html_path
            self.open_path_button.set_state("normal")
        else:
            self.latest_trace_html = None
            self.open_path_button.set_state("disabled")
        if image_saved:
            self._append_console(f"Saved flying path image to {image_path}")
        if html_saved:
            self._append_console(f"Saved interactive 3D path to {html_path}")
        elif image_saved:
            reason = PLOTLY_IMPORT_ERROR or "Plotly export did not create an HTML file"
            self._append_console(f"WARNING: 3D HTML path export skipped: {reason}")

    def _save_path_image(self, output_path: pathlib.Path) -> bool:
        if Image is None or ImageDraw is None:
            return False

        width = 1280
        height = 900
        scene = self._build_path_scene(width, height)
        if scene is None:
            return False

        try:
            image = Image.new("RGBA", (width, height), PALETTE["canvas_bg"])
            draw = ImageDraw.Draw(image, "RGBA")

            draw.polygon(scene["floor_polygon"], fill=(16, 16, 16, 255), outline=(27, 27, 27, 255))

            start_shadow = scene["start_shadow"]
            draw.line([start_shadow, scene["x_axis"]], fill=(80, 80, 80, 150), width=4)
            draw.line([start_shadow, scene["y_axis"]], fill=(80, 80, 80, 150), width=4)
            draw.line([start_shadow, scene["z_axis"]], fill=(64, 64, 64, 170), width=4)
            draw.text((start_shadow[0] + 14, start_shadow[1] + 14), "START", fill=(138, 138, 138, 220))
            draw.text((scene["x_axis_label"][0] + 12, scene["x_axis_label"][1] + 2), "X", fill=(150, 150, 150, 120))
            draw.text((scene["y_axis_label"][0] + 12, scene["y_axis_label"][1] + 2), "Y", fill=(150, 150, 150, 120))

            shadow_points = scene["shadow_points"]
            path_points = scene["path_points"]
            if len(shadow_points) >= 2:
                draw.line(shadow_points, fill=(58, 36, 23, 220), width=16)

            step = max(1, len(path_points) // 10)
            for idx in range(step, len(path_points), step):
                draw.line([shadow_points[idx], path_points[idx]], fill=(74, 56, 45, 180), width=2)

            if len(path_points) >= 2:
                draw.line(path_points, fill=(252, 76, 2, 255), width=9)

            head_x, head_y = path_points[-1]
            shadow_x, shadow_y = shadow_points[-1]
            draw.line([(shadow_x, shadow_y), (head_x, head_y)], fill=(109, 75, 58, 220), width=3)
            draw.ellipse((head_x - 26, head_y - 26, head_x + 26, head_y + 26), outline=(255, 138, 117, 160), width=4)
            draw.ellipse((head_x - 16, head_y - 16, head_x + 16, head_y + 16), outline=(255, 90, 79, 220), width=5)
            draw.ellipse((head_x - 9, head_y - 9, head_x + 9, head_y + 9), fill=(255, 59, 48, 255), outline=(255, 212, 206, 255), width=2)

            image.convert("RGB").save(output_path, format="JPEG", quality=92)
        except Exception as exc:
            self._append_console(f"WARNING: Flying path image export failed: {exc}")
            return False
        return output_path.exists()

    def _save_path_html(self, output_path: pathlib.Path) -> bool:
        global PLOTLY_IMPORT_ERROR
        plotly_go = ensure_plotly()
        if plotly_go is None or not self.trace_points:
            return False

        xs = [point[0] for point in self.trace_points]
        ys = [point[1] for point in self.trace_points]
        zs = [point[2] for point in self.trace_points]

        fig = plotly_go.Figure()
        fig.add_trace(
            plotly_go.Scatter3d(
                x=xs,
                y=ys,
                z=zs,
                mode="lines",
                line={"color": "#FC4C02", "width": 6},
                name="Trace",
            )
        )
        fig.add_trace(
            plotly_go.Scatter3d(
                x=[xs[-1]],
                y=[ys[-1]],
                z=[zs[-1]],
                mode="markers",
                marker={"size": 7, "color": "#ff3b30"},
                name="Current Position",
            )
        )
        fig.add_trace(
            plotly_go.Scatter3d(
                x=[0.0],
                y=[0.0],
                z=[0.0],
                mode="markers",
                marker={"size": 5, "color": "#9a9a9a"},
                name="Start",
            )
        )

        fig.update_layout(
            template="plotly_dark",
            title="Crazyflie Flying Path",
            paper_bgcolor="#080808",
            plot_bgcolor="#080808",
            scene={
                "xaxis": {
                    "title": "X (cm)",
                    "backgroundcolor": "rgba(0,0,0,0)",
                    "gridcolor": "rgba(150,150,150,0.18)",
                    "zerolinecolor": "rgba(180,180,180,0.28)",
                },
                "yaxis": {
                    "title": "Y (cm)",
                    "backgroundcolor": "rgba(0,0,0,0)",
                    "gridcolor": "rgba(150,150,150,0.18)",
                    "zerolinecolor": "rgba(180,180,180,0.28)",
                },
                "zaxis": {
                    "title": "Z (cm)",
                    "backgroundcolor": "rgba(0,0,0,0)",
                    "gridcolor": "rgba(150,150,150,0.18)",
                    "zerolinecolor": "rgba(180,180,180,0.28)",
                },
                "aspectmode": "data",
                "camera": {
                    "eye": {"x": 1.45, "y": -1.6, "z": 1.15},
                },
            },
            margin={"l": 0, "r": 0, "t": 50, "b": 0},
            legend={"orientation": "h", "yanchor": "bottom", "y": 0.98, "xanchor": "right", "x": 1.0},
        )
        try:
            fig.write_html(str(output_path), include_plotlyjs=True, full_html=True)
        except Exception as exc:
            PLOTLY_IMPORT_ERROR = str(exc)
            return False
        return output_path.exists()

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
                mission_snapshot = MissionSnapshot(**payload)
                self._play_human_sfx(self.last_human_sound_snapshot, mission_snapshot)
                self.last_human_sound_snapshot = mission_snapshot
                self.mission = mission_snapshot
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

        self.human_led_canvas.itemconfigure("led", fill=led_color)
        self.human_led_canvas.itemconfigure("led_text", text=human_text)
        self.fresh_value.configure(text=f"Fresh: {self.mission.fresh}")
        self.stable_value.configure(text=f"Stable: {self.mission.stable}")
        uart_color = "#34c82a" if self.mission.fresh else "#d93025"
        self.uart_status_canvas.itemconfigure("uart_led", fill=uart_color)
        self.max_temp_value.configure(text=f"Max Temp: {self.mission.max_temp_c:.2f} C")
        self.therm_value.configure(text=f"Thermistor: {self.mission.therm_c:.2f} C")

    def _render_flow(self) -> None:
        self.delta_value.configure(text=f"({self.flow.delta_x}, {self.flow.delta_y})")
        self.shutter_value.configure(text=str(self.flow.shutter))
        self.vel_value.configure(text=f"({self.flow.vel_x:.2f}, {self.flow.vel_y:.2f}) m/s")
        self.cmd_value.configure(text=f"({self.flow.cmd_x:.2f}, {self.flow.cmd_y:.2f}) m/s")
        if self.show_trace_position and self.flow.pos_valid:
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
    parser.add_argument("--uri", help="Crazyflie link URI. If omitted, the dashboard starts disconnected and waits for Scan or manual entry.")
    parser.add_argument("--cache", default="/Users/zhangzehua/Desktop/fyp/cache", help="TOC cache directory")
    parser.add_argument(
        "--csv-dir",
        default=str(SCRIPT_DIR / "telemetry_logs"),
        help="Directory where realtime telemetry CSV files will be written",
    )
    parser.add_argument("--log-period-ms", type=int, default=250, help="Telemetry log period in ms")
    parser.add_argument("--no-auto-connect", dest="auto_connect", action="store_false", help="Open the window without auto-connecting when --uri is provided")
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
