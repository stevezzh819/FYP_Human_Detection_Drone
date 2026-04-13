#!/usr/bin/env python3
"""Live AMG8833 heatmap viewer from ESP32 console output.

This viewer now follows the same rendering approach as the reference
`/Users/zhangzehua/Desktop/heatmap_viewer.py`:

- matplotlib `imshow`
- `inferno` colormap
- `bilinear` interpolation
- fixed default heat scale
- per-cell temperature labels

It accepts either serial payload format:

AMG_FRAME,<timestamp_ms>,<thermistor_c>,<ambient_c>,<max_temp_c>,<human>,<confidence>,<direction>,<p0>...<p63>
DATA,<p0>...<p63>
"""

from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.patches import FancyBboxPatch
from serial.tools import list_ports


TEMP_MIN = 20.0
TEMP_MAX = 35.0
AUTO_SCALE = False
PLOT_BG = "#121212"
AXIS_BG = "#121212"
TEXT_LIGHT = "#ffffff"
TEXT_MID = "#aaaaaa"
TEXT_DIM = "#666666"
SERIAL_TIMEOUT_S = 1.0


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


@dataclass
class ReaderState:
    requested_port: Optional[str]
    active_port: Optional[str] = None
    connected: bool = False
    last_error: str = "Waiting for serial port..."
    last_frame_time: float = 0.0
    blob_size: Optional[int] = None


BLOB_RE = re.compile(r"blob=(\d+)")


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

        if any(tag in desc for tag in ("esp32", "usb serial", "usb jtag", "serial debug unit", "cp210", "wch", "ch340")):
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
    parts = text.split(",")

    if text.startswith("AMG_FRAME,"):
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
    elif text.startswith("DATA,"):
        if len(parts) != 65:
            return None

        try:
            pixels = [float(value) for value in parts[1:65]]
        except ValueError:
            return None

        ambient_c = float(sum(pixels) / len(pixels))
        max_temp_c = float(max(pixels))
        timestamp_ms = int(time.time() * 1000.0)
        thermistor_c = ambient_c
        human = 0
        confidence = 0
        direction = 0
    else:
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


def parse_blob_size(line: str) -> Optional[int]:
    match = BLOB_RE.search(line)
    if not match:
        return None
    try:
        return int(match.group(1))
    except ValueError:
        return None


def serial_reader(requested_port: Optional[str],
                  baudrate: int,
                  event_queue: queue.Queue[Frame],
                  reader_state: ReaderState,
                  stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        port = requested_port or guess_port()
        reader_state.active_port = port

        if not port:
            reader_state.connected = False
            reader_state.last_error = "No ESP32 serial port detected"
            stop_event.wait(1.0)
            continue

        try:
            ser = serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT_S)
            reader_state.connected = True
            reader_state.last_error = ""
            print(f"[serial] Opened {port} @ {baudrate} baud")
        except serial.SerialException as exc:
            reader_state.connected = False
            reader_state.last_error = str(exc)
            print(f"[serial] ERROR: {exc}")
            stop_event.wait(1.0)
            continue

        try:
            while not stop_event.is_set():
                try:
                    raw = ser.readline().decode("utf-8", errors="ignore").strip()
                except serial.SerialException as exc:
                    reader_state.connected = False
                    reader_state.last_error = str(exc)
                    print(f"[serial] ERROR: {exc}")
                    break

                if not raw:
                    continue

                frame = parse_frame_line(raw)
                if frame is not None:
                    reader_state.last_frame_time = time.time()
                    try:
                        event_queue.put_nowait(frame)
                    except queue.Full:
                        pass
                else:
                    blob_size = parse_blob_size(raw)
                    if blob_size is not None:
                        reader_state.blob_size = blob_size

                    if raw.startswith("I (") or raw.startswith("W (") or raw.startswith("E ("):
                        continue

                    print(f"[esp32] {raw}")
        finally:
            reader_state.connected = False
            try:
                ser.close()
            except Exception:
                pass

        stop_event.wait(0.5)


def main() -> int:
    parser = argparse.ArgumentParser(description="Live AMG8833 heatmap viewer from ESP32 serial output")
    parser.add_argument("port_arg", nargs="?", help="Optional serial port like /dev/cu.usbmodem101")
    parser.add_argument("--port", help="Serial port for the ESP32 console, for example /dev/cu.usbmodem101")
    parser.add_argument("--baudrate", "-b", type=int, default=115200, help="ESP32 console baud rate")
    parser.add_argument("--scale-min", type=float, default=TEMP_MIN, help="Fixed minimum temperature scale")
    parser.add_argument("--scale-max", type=float, default=TEMP_MAX, help="Fixed maximum temperature scale")
    parser.add_argument("--auto-scale", action="store_true", help="Scale each frame to its own min/max instead of using a fixed thermal range")
    args = parser.parse_args()

    requested_port = args.port or args.port_arg
    initial_port = requested_port or guess_port()
    if not initial_port:
        print("No ESP32 serial port detected right now. The viewer will keep retrying.", file=sys.stderr)

    data_queue: queue.Queue[Frame] = queue.Queue(maxsize=5)
    stop_event = threading.Event()
    reader_state = ReaderState(requested_port=requested_port, active_port=initial_port)

    threading.Thread(
        target=serial_reader,
        args=(requested_port, args.baudrate, data_queue, reader_state, stop_event),
        daemon=True,
    ).start()

    latest_frame: Optional[Frame] = None
    total_frames = 0

    fig, ax = plt.subplots(figsize=(6, 6.5))
    fig.patch.set_facecolor(PLOT_BG)
    ax.set_facecolor(AXIS_BG)
    plt.subplots_adjust(top=0.80, bottom=0.18)

    dummy = np.full((8, 8), (args.scale_min + args.scale_max) / 2.0, dtype=np.float32)
    im = ax.imshow(
        dummy,
        cmap="inferno",
        vmin=args.scale_min,
        vmax=args.scale_max,
        interpolation="bilinear",
        origin="lower",
    )

    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("°C", color=TEXT_LIGHT)
    cbar.ax.yaxis.set_tick_params(color=TEXT_LIGHT)
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color=TEXT_LIGHT)

    ax.set_xticks(range(8))
    ax.set_yticks(range(8))
    ax.tick_params(colors="#444444", labelsize=7)

    title = fig.suptitle("Waiting for data...", color=TEXT_LIGHT, fontsize=12, y=0.97)

    port_ax = fig.add_axes([0.125, 0.10, 0.75, 0.055])
    port_ax.set_axis_off()
    port_box = port_ax.add_patch(
        FancyBboxPatch(
            (0, 0),
            1,
            1,
            boxstyle="round,pad=0.05",
            facecolor="#2a2a2a",
            transform=port_ax.transAxes,
            clip_on=False,
        )
    )
    port_txt = port_ax.text(
        0.5,
        0.5,
        f"Port: {initial_port or 'auto-detect'} @ {args.baudrate}",
        ha="center",
        va="center",
        color=TEXT_MID,
        fontsize=9,
        transform=port_ax.transAxes,
    )

    status_ax = fig.add_axes([0.125, 0.03, 0.75, 0.055])
    status_ax.set_axis_off()
    status_box = status_ax.add_patch(
        FancyBboxPatch(
            (0, 0),
            1,
            1,
            boxstyle="round,pad=0.05",
            facecolor="#555555",
            transform=status_ax.transAxes,
            clip_on=False,
        )
    )
    status_txt = status_ax.text(
        0.5,
        0.5,
        "Waiting for heat frames...",
        ha="center",
        va="center",
        color=TEXT_LIGHT,
        fontsize=9,
        fontweight="bold",
        transform=status_ax.transAxes,
    )

    debug_txt = fig.text(
        0.5,
        0.835,
        "Blob size: --",
        ha="center",
        color=TEXT_MID,
        fontsize=9,
    )

    fig.text(
        0.5,
        0.005,
        "Viewer follows the reference: inferno palette, fixed thermal range, bilinear interpolation, per-cell labels",
        ha="center",
        color=TEXT_DIM,
        fontsize=7.5,
    )

    cell_texts = [
        ax.text(col, row, "", ha="center", va="center", fontsize=6, zorder=5)
        for row in range(8)
        for col in range(8)
    ]

    def update(_frame_index: int) -> None:
        nonlocal latest_frame, total_frames

        newest: Optional[Frame] = None
        try:
            while True:
                newest = data_queue.get_nowait()
        except queue.Empty:
            pass

        active_port = reader_state.active_port or "auto-detect"
        port_txt.set_text(f"Port: {active_port} @ {args.baudrate}")
        blob_text = "--" if reader_state.blob_size is None else str(reader_state.blob_size)
        debug_txt.set_text(f"Blob size: {blob_text}")

        if newest is None:
            if reader_state.connected:
                status_box.set_facecolor("#6d4c41")
                age_s = time.time() - reader_state.last_frame_time if reader_state.last_frame_time else None
                if age_s is None:
                    status_txt.set_text("Connected  |  waiting for AMG_FRAME data")
                else:
                    status_txt.set_text(f"Connected  |  last frame {age_s:.1f}s ago")
            else:
                status_box.set_facecolor("#8e24aa")
                status_txt.set_text(reader_state.last_error or "Waiting for serial port...")
            return

        latest_frame = newest
        total_frames += 1
        grid = np.asarray(latest_frame.pixels, dtype=np.float32).reshape(8, 8)

        if args.auto_scale:
            im.set_clim(float(np.min(grid)), float(np.max(grid)))
        else:
            im.set_clim(args.scale_min, args.scale_max)
        im.set_data(grid)

        clim = im.get_clim()
        for idx, txt in enumerate(cell_texts):
            row, col = divmod(idx, 8)
            value = grid[row, col]
            txt.set_text(f"{value:.1f}")
            norm = (value - clim[0]) / (clim[1] - clim[0] + 1e-6)
            txt.set_color("#000000" if norm > 0.55 else "#ffffff")

        if latest_frame.human:
            status_box.set_facecolor("#e53935")
            status_txt.set_text(
                f"HUMAN  |  conf {latest_frame.confidence}%  |  dir {direction_name(latest_frame.direction)}  |  blob {blob_text}"
            )
        else:
            status_box.set_facecolor("#0288d1")
            status_txt.set_text(
                f"NO HUMAN  |  ambient {latest_frame.ambient_c:.1f} C  |  max {latest_frame.max_temp_c:.1f} C  |  blob {blob_text}"
            )

        port_box.set_facecolor("#1f2d3d" if latest_frame.human else "#2a2a2a")
        port_txt.set_color(TEXT_LIGHT if latest_frame.human else TEXT_MID)

        title.set_text(
            f"AMG8833  |  {grid.min():.1f} C - {grid.max():.1f} C"
            f"  |  frames: {total_frames}"
        )

    anim = animation.FuncAnimation(fig, update, interval=110, cache_frame_data=False)

    try:
        plt.show()
    finally:
        stop_event.set()
        print("[viewer] Closed")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
