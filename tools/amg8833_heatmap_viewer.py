#!/usr/bin/env python3
"""Live AMG8833 8x8 heatmap viewer from ESP32 console output.

This tool listens for lines emitted by the ESP32 firmware in the format:

AMG_FRAME,<timestamp_ms>,<thermistor_c>,<ambient_c>,<max_temp_c>,<human>,<confidence>,<direction>,<p0>...<p63>

It opens a separate Tk window and renders the 8x8 thermal frame in real time.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import tkinter as tk
from dataclasses import dataclass
from typing import Optional

import serial
from serial.tools import list_ports


@dataclass
class Frame:
    timestamp_ms: int
    thermistor_c: float
    ambient_c: float
    max_temp_c: float
    human: int
    confidence: int
    direction: int
    pixels: list[float]


def direction_name(direction: int) -> str:
    if direction < 0:
        return "Left"
    if direction > 0:
        return "Right"
    return "Center"


def guess_port() -> Optional[str]:
    preferred = []
    fallback = []

    for port in list_ports.comports():
        name = (port.device or "").lower()
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()

        if any(tag in desc for tag in ("esp32", "usb serial", "cp210", "wch", "ch340")):
            preferred.append(port.device)
            continue

        if any(tag in name for tag in ("usbmodem", "usbserial", "wchusbserial", "cu.usb")):
            fallback.append(port.device)

    if preferred:
        return preferred[0]
    if fallback:
        return fallback[0]
    return None


def parse_frame_line(line: str) -> Optional[Frame]:
    text = line.strip()
    if not text.startswith("AMG_FRAME,"):
        return None

    parts = text.split(",")
    if len(parts) != 72:
        return None

    try:
        timestamp_ms = int(parts[1])
        thermistor_c = float(parts[2])
        ambient_c = float(parts[3])
        max_temp_c = float(parts[4])
        human = int(parts[5])
        confidence = int(parts[6])
        direction = int(parts[7])
        pixels = [float(value) for value in parts[8:72]]
    except ValueError:
        return None

    return Frame(
        timestamp_ms=timestamp_ms,
        thermistor_c=thermistor_c,
        ambient_c=ambient_c,
        max_temp_c=max_temp_c,
        human=human,
        confidence=confidence,
        direction=direction,
        pixels=pixels,
    )


def clamp01(value: float) -> float:
    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0
    return value


def lerp(a: float, b: float, t: float) -> float:
    return a + ((b - a) * t)


def rgb_to_hex(r: int, g: int, b: int) -> str:
    return f"#{r:02x}{g:02x}{b:02x}"


def heat_color(norm: float) -> str:
    norm = clamp01(norm)
    anchors = [
        (0.00, (18, 32, 87)),
        (0.25, (35, 114, 196)),
        (0.50, (48, 191, 191)),
        (0.75, (251, 191, 36)),
        (1.00, (196, 38, 38)),
    ]

    for idx in range(len(anchors) - 1):
        left_pos, left_rgb = anchors[idx]
        right_pos, right_rgb = anchors[idx + 1]
        if norm <= right_pos:
            local = (norm - left_pos) / (right_pos - left_pos) if right_pos > left_pos else 0.0
            r = int(round(lerp(left_rgb[0], right_rgb[0], local)))
            g = int(round(lerp(left_rgb[1], right_rgb[1], local)))
            b = int(round(lerp(left_rgb[2], right_rgb[2], local)))
            return rgb_to_hex(r, g, b)

    return rgb_to_hex(*anchors[-1][1])


class HeatmapViewer:
    def __init__(self, port: str, baudrate: int, scale_min: Optional[float], scale_max: Optional[float]) -> None:
        self.port = port
        self.baudrate = baudrate
        self.fixed_scale_min = scale_min
        self.fixed_scale_max = scale_max
        self.frames: queue.Queue[Frame] = queue.Queue(maxsize=4)
        self.stop_event = threading.Event()
        self.root = tk.Tk()
        self.root.title(f"AMG8833 Heatmap Viewer [{self.port}]")
        self.root.configure(bg="#101010")
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.status_var = tk.StringVar(value="Waiting for AMG8833 frames...")
        self.scale_var = tk.StringVar(value="Scale: --")

        self.header = tk.Label(
            self.root,
            textvariable=self.status_var,
            bg="#101010",
            fg="#f3f3f3",
            font=("Helvetica", 13, "bold"),
            justify="left",
            anchor="w",
        )
        self.header.pack(fill="x", padx=12, pady=(12, 6))

        self.scale_label = tk.Label(
            self.root,
            textvariable=self.scale_var,
            bg="#101010",
            fg="#b6b6b6",
            font=("Helvetica", 10),
            justify="left",
            anchor="w",
        )
        self.scale_label.pack(fill="x", padx=12, pady=(0, 10))

        self.canvas = tk.Canvas(self.root, width=520, height=520, bg="#0b0b0b", highlightthickness=0)
        self.canvas.pack(padx=12, pady=(0, 12))

        self.cells = []
        self.cell_text = []
        cell_size = 60
        margin = 20
        for row in range(8):
            row_cells = []
            row_text = []
            for col in range(8):
                x0 = margin + (col * cell_size)
                y0 = margin + (row * cell_size)
                x1 = x0 + cell_size - 4
                y1 = y0 + cell_size - 4
                rect = self.canvas.create_rectangle(x0, y0, x1, y1, fill="#202020", outline="#3a3a3a", width=1)
                text = self.canvas.create_text(
                    (x0 + x1) / 2,
                    (y0 + y1) / 2,
                    text="--",
                    fill="#ffffff",
                    font=("Helvetica", 9, "bold"),
                )
                row_cells.append(rect)
                row_text.append(text)
            self.cells.append(row_cells)
            self.cell_text.append(row_text)

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.root.after(50, self._drain_frames)

    def _close(self) -> None:
        self.stop_event.set()
        self.root.destroy()

    def _reader_loop(self) -> None:
        try:
            with serial.Serial(self.port, self.baudrate, timeout=1.0) as ser:
                while not self.stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("utf-8", errors="ignore")
                    except UnicodeDecodeError:
                        continue
                    frame = parse_frame_line(line)
                    if frame is None:
                        continue
                    while not self.frames.empty():
                        try:
                            self.frames.get_nowait()
                        except queue.Empty:
                            break
                    self.frames.put(frame)
        except serial.SerialException as exc:
            self.status_var.set(f"Serial error on {self.port}: {exc}")

    def _drain_frames(self) -> None:
        latest = None
        while True:
            try:
                latest = self.frames.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            self._render_frame(latest)

        if not self.stop_event.is_set():
            self.root.after(50, self._drain_frames)

    def _render_frame(self, frame: Frame) -> None:
        if self.fixed_scale_min is not None:
            scale_min = self.fixed_scale_min
        else:
            scale_min = min(frame.pixels)

        if self.fixed_scale_max is not None:
            scale_max = self.fixed_scale_max
        else:
            scale_max = max(frame.pixels)

        if scale_max <= scale_min:
            scale_max = scale_min + 1.0

        human_text = "YES" if frame.human else "NO"
        self.status_var.set(
            "Human: {} | Conf: {}% | Dir: {} | Therm: {:.2f} C | Amb: {:.2f} C | Max: {:.2f} C | t={} ms".format(
                human_text,
                frame.confidence,
                direction_name(frame.direction),
                frame.thermistor_c,
                frame.ambient_c,
                frame.max_temp_c,
                frame.timestamp_ms,
            )
        )
        self.scale_var.set(f"Scale: {scale_min:.2f} C to {scale_max:.2f} C")

        for row in range(8):
            for col in range(8):
                idx = ((7 - row) * 8) + col
                value = frame.pixels[idx]
                norm = (value - scale_min) / (scale_max - scale_min)
                color = heat_color(norm)
                self.canvas.itemconfigure(self.cells[row][col], fill=color)
                self.canvas.itemconfigure(self.cell_text[row][col], text=f"{value:.1f}")

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Live AMG8833 heatmap viewer from ESP32 serial output")
    parser.add_argument("--port", help="Serial port for the ESP32 console, for example /dev/cu.usbmodem*")
    parser.add_argument("--baudrate", type=int, default=115200, help="ESP32 console baud rate")
    parser.add_argument("--scale-min", type=float, help="Fixed minimum temperature scale")
    parser.add_argument("--scale-max", type=float, help="Fixed maximum temperature scale")
    args = parser.parse_args()

    port = args.port or guess_port()
    if not port:
        print("No ESP32 serial port detected. Pass --port explicitly.", file=sys.stderr)
        return 1

    viewer = HeatmapViewer(
        port=port,
        baudrate=args.baudrate,
        scale_min=args.scale_min,
        scale_max=args.scale_max,
    )
    viewer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
