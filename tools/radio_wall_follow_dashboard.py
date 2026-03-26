#!/usr/bin/env python3
"""Tk dashboard for the Crazyflie wall-follow mission over Crazyradio."""

from __future__ import annotations

import argparse
import pathlib
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
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


@dataclass
class RangeSnapshot:
    timestamp_ms: int = 0
    front: int = 32766
    left: int = 32766
    right: int = 32766
    back: int = 32766
    up: int = 32766
    vbat: float = 0.0


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


class RadioSession(threading.Thread):
    def __init__(
        self,
        preferred_uri: Optional[str],
        cache_dir: str,
        log_period_ms: int,
        event_queue: "queue.Queue[tuple[str, Dict[str, object]]]",
    ) -> None:
        super().__init__(name="cf-radio-session", daemon=True)
        self.preferred_uri = preferred_uri
        self.cache_dir = cache_dir
        self.log_period_ms = log_period_ms
        self.event_queue = event_queue
        self.command_queue: "queue.Queue[tuple[str, Optional[int]]]" = queue.Queue()
        self.stop_event = threading.Event()

    def request_active(self, active: bool) -> None:
        self.command_queue.put(("set_active", 1 if active else 0))

    def shutdown(self) -> None:
        self.stop_event.set()
        self.command_queue.put(("stop", None))

    def _emit(self, event_type: str, **payload: object) -> None:
        self.event_queue.put((event_type, payload))

    def run(self) -> None:
        ok, message = check_crazyradio_access()
        if not ok:
            self._emit("status", level="error", message=message)
            self._emit("disconnected")
            return

        cflib.crtp.init_drivers()

        try:
            uri = discover_uri(self.preferred_uri)
        except RuntimeError as exc:
            self._emit("status", level="error", message=str(exc))
            self._emit("disconnected")
            return

        cf = Crazyflie(rw_cache=self.cache_dir)
        logconfs: list[LogConfig] = []
        line_buffer: list[str] = []

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
            self._emit(
                "mission",
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

        def on_flow(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            self._emit(
                "flow",
                timestamp_ms=ts,
                delta_x=int(data["motion.deltaX"]),
                delta_y=int(data["motion.deltaY"]),
                shutter=int(data["motion.shutter"]),
                vel_x=float(data["stateEstimate.vx"]),
                vel_y=float(data["stateEstimate.vy"]),
                cmd_x=float(data["app.cmdVelX"]),
                cmd_y=float(data["app.cmdVelY"]),
            )

        def on_range(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
            self._emit(
                "range",
                timestamp_ms=ts,
                front=int(data["range.front"]),
                left=int(data["range.left"]),
                right=int(data["range.right"]),
                back=int(data["range.back"]),
                up=int(data["range.up"]),
                vbat=float(data["pm.vbat"]),
            )

        def set_active_with_retry(active: int) -> None:
            last_error: Optional[Exception] = None
            for _ in range(5):
                try:
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
            with SyncCrazyflie(uri, cf=cf):
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

                logconfs.extend([mission_cfg, flow_cfg, range_cfg])
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
                        set_active_with_retry(int(value or 0))
                        self._emit("status", level="info", message=f"app.active={int(value or 0)}")
                    elif command == "stop":
                        break
        except Exception as exc:
            self._emit("status", level="error", message=f"Radio session failed: {exc}")
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

            self._emit("disconnected")


class DashboardApp:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.event_queue: "queue.Queue[tuple[str, Dict[str, object]]]" = queue.Queue()
        self.worker: Optional[RadioSession] = None
        self.connected = False
        self.closing = False
        self.mission = MissionSnapshot()
        self.flow = FlowSnapshot()
        self.range = RangeSnapshot()
        self.console_lines: list[str] = []
        self.uri_var = tk.StringVar(value=args.uri or "radio://0/60/2M")
        self.status_var = tk.StringVar(value="Waiting to connect")
        self.connection_var = tk.StringVar(value="Disconnected")

        self._configure_root()
        self._build_layout()
        self._render_all()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(60, self._pump_events)
        if self.args.auto_connect:
            self.root.after(150, self.connect)

    def _configure_root(self) -> None:
        self.root.title("Crazyflie Mission Dashboard")
        self.root.geometry("980x760")
        self.root.minsize(900, 700)
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
        self.connect_button = ColorButton(
            buttons,
            text="Connect",
            command=self.connect,
            bg="#000000",
            fg="#ffffff",
            active_bg="#1a1a1a",
            active_fg="#ffffff",
            width=12,
            font=("Helvetica", 11, "bold"),
        )
        self.connect_button.grid(row=0, column=0, padx=4)
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
        self.start_button.grid(row=0, column=1, padx=4)
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
        self.stop_button.grid(row=0, column=2, padx=4)

        status_row = tk.Frame(self.root, bg=PALETTE["app_bg"])
        status_row.pack(fill="x", padx=18, pady=(0, 12))
        tk.Label(status_row, text="Connection", bg=PALETTE["app_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11, "bold")).pack(side="left")
        tk.Label(status_row, textvariable=self.connection_var, bg=PALETTE["app_bg"], fg=PALETTE["text_main"], font=("Helvetica", 11)).pack(side="left", padx=(10, 24))
        tk.Label(status_row, textvariable=self.status_var, bg=PALETTE["app_bg"], fg=PALETTE["text_muted"], font=("Helvetica", 11), anchor="w").pack(side="left")

        cards = tk.Frame(self.root, bg=PALETTE["app_bg"])
        cards.pack(fill="both", expand=True, padx=18, pady=(0, 18))
        cards.grid_columnconfigure(0, weight=1)
        cards.grid_columnconfigure(1, weight=1)
        cards.grid_rowconfigure(0, weight=1)
        cards.grid_rowconfigure(1, weight=1)

        self.mission_frame = self._panel(cards, "Mission")
        self.mission_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 9), pady=(0, 9))
        self._build_mission_panel()

        self.human_frame = self._panel(cards, "Human Detection")
        self.human_frame.grid(row=0, column=1, sticky="nsew", padx=(9, 0), pady=(0, 9))
        self._build_human_panel()

        self.flow_frame = self._panel(cards, "Flow")
        self.flow_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 9), pady=(9, 0))
        self._build_flow_panel()

        self.range_frame = self._panel(cards, "Range Sector")
        self.range_frame.grid(row=1, column=1, sticky="nsew", padx=(9, 0), pady=(9, 0))
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

        self.human_led_canvas = tk.Canvas(body, width=180, height=180, bg=PALETTE["panel_bg"], highlightthickness=0)
        self.human_led_canvas.pack(pady=(6, 12))
        self.human_led_canvas.create_oval(20, 20, 160, 160, fill=PALETTE["human_off"], outline=PALETTE["text_muted"], width=4, tags="led")
        self.human_led_canvas.create_text(90, 90, text="NO\nHUMAN", fill=PALETTE["text_main"], font=("Helvetica", 18, "bold"), tags="led_text")

        self.human_status = tk.Label(body, text="Waiting for telemetry", bg=PALETTE["panel_bg"], fg=PALETTE["text_main"], font=("Helvetica", 13, "bold"))
        self.human_status.pack(pady=(0, 10))
        self.max_temp_value = tk.Label(body, text="Max Temp: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.max_temp_value.pack(anchor="w")
        self.therm_value = tk.Label(body, text="Thermistor: --", bg=PALETTE["panel_bg"], fg=PALETTE["text_dim"], font=("Helvetica", 11))
        self.therm_value.pack(anchor="w")

    def _build_flow_panel(self) -> None:
        body = tk.Frame(self.flow_frame, bg=PALETTE["panel_bg"])
        body.pack(fill="both", expand=True, padx=4, pady=(0, 12))

        self.delta_value = self._value_pair(body, 0, "Delta")
        self.shutter_value = self._value_pair(body, 1, "Shutter")
        self.vel_value = self._value_pair(body, 2, "Velocity")
        self.cmd_value = self._value_pair(body, 3, "Command")
        self.flow_age_value = self._value_pair(body, 4, "Timestamp")

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
        if self.worker and self.worker.is_alive():
            return

        preferred_uri = self.uri_var.get().strip() or None
        self.status_var.set("Connecting...")
        self.connection_var.set("Connecting")
        self._append_console("Connecting to Crazyflie radio session")
        self.connect_button.set_state("disabled")

        self.worker = RadioSession(
            preferred_uri=preferred_uri,
            cache_dir=self.args.cache,
            log_period_ms=self.args.log_period_ms,
            event_queue=self.event_queue,
        )
        self.worker.start()

    def start_mission(self) -> None:
        if not self.worker or not self.connected:
            return
        self.worker.request_active(True)
        self._append_console("Start button pressed")

    def stop_mission(self) -> None:
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
        self.connection_var.set(f"Connected: {payload.get('uri', '')}")
        self.connect_button.set_state("disabled")
        self.start_button.set_state("normal")
        self.stop_button.set_state("normal")
        self._append_console(f"Connected to {payload.get('uri', '')}")

    def _handle_disconnected(self) -> None:
        self.connected = False
        self.connection_var.set("Disconnected")
        self.connect_button.set_state("normal")
        self.start_button.set_state("disabled")
        self.stop_button.set_state("disabled")
        if not self.closing:
            self.status_var.set("Disconnected")
            self._append_console("Radio session stopped")

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
            elif event_type == "mission":
                self.mission = MissionSnapshot(**payload)
                self._render_mission()
                self._render_human()
            elif event_type == "flow":
                self.flow = FlowSnapshot(**payload)
                self._render_flow()
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
        status_text = "Detected" if self.mission.human else "No human"
        if self.mission.human and self.mission.stable:
            status_text = "Stable detection"
        elif self.mission.human and self.mission.fresh:
            status_text = "Fresh detection"

        self.human_led_canvas.itemconfigure("led", fill=led_color)
        self.human_led_canvas.itemconfigure("led_text", text=human_text)
        self.human_status.configure(text=status_text)
        self.fresh_value.configure(text=f"Fresh: {self.mission.fresh}")
        self.stable_value.configure(text=f"Stable: {self.mission.stable}")
        self.max_temp_value.configure(text=f"Max Temp: {self.mission.max_temp_c:.2f} C")
        self.therm_value.configure(text=f"Thermistor: {self.mission.therm_c:.2f} C")

    def _render_flow(self) -> None:
        self.delta_value.configure(text=f"({self.flow.delta_x}, {self.flow.delta_y})")
        self.shutter_value.configure(text=str(self.flow.shutter))
        self.vel_value.configure(text=f"({self.flow.vel_x:.2f}, {self.flow.vel_y:.2f}) m/s")
        self.cmd_value.configure(text=f"({self.flow.cmd_x:.2f}, {self.flow.cmd_y:.2f}) m/s")
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
    parser.add_argument("--uri", help="Crazyflie link URI. Defaults to radio://0/60/2M.")
    parser.add_argument("--cache", default="/Users/zhangzehua/Desktop/fyp/cache", help="TOC cache directory")
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
