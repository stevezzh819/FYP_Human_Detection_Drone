# Autonomous Indoor Nano-Drone Navigation with Human Detection

## Overview

This repository contains a complete Final Year Project system built around a **Crazyflie 2.1**, an **ESP32-C3 SuperMini**, and an **AMG8833 8x8 thermal sensor**.

The project combines:

- **Crazyflie firmware** for wall-following flight logic and mission control
- **ESP32 firmware** for thermal sensing and human-detection inference
- **UART communication** between ESP32 and Crazyflie
- **Python desktop tools** for mission control, telemetry logging, path visualization, and thermal debugging

At a high level, the system works as follows:

1. The **AMG8833** sends thermal pixel data to the **ESP32-C3** over **I2C**.
2. The **ESP32** processes the thermal frame, estimates human presence, and sends a compact detection packet to the **Crazyflie** over **UART**.
3. The **Crazyflie firmware** receives and parses the UART packet, then uses the result inside the mission logic in `wall_following.c`.
4. A **Python dashboard** connects to the Crazyflie through **Crazyradio / CRTP**, displays telemetry, logs flight data to CSV, exports path plots, and supports mission start/stop control.

This repository is therefore not just firmware, but a full **embedded + flight-control + telemetry visualization** workflow.

---

## Main Project Components

### 1. Crazyflie Firmware
Location:
- `crazyflie/crazyflie-firmware/`

Main purpose:
- Runs the flight control stack and application-level wall-following mission
- Receives ESP32 human-detection data through UART
- Exposes telemetry to the Python dashboard through Crazyflie logging / CRTP

Important files:
- `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c`
- `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c`
- `crazyflie/crazyflie-firmware/src/modules/interface/esp_uart_bridge.h`
- `crazyflie/crazyflie-firmware/src/drivers/src/uart2.c`

### 2. ESP32 Firmware
Location:
- `espcontrol/crazyflie_esp/`

Main purpose:
- Reads AMG8833 thermal frames over I2C
- Runs human-detection logic
- Sends detection results to Crazyflie over UART
- Can stream thermal frames to the desktop heatmap viewer over USB Serial/JTAG

Important files:
- `espcontrol/crazyflie_esp/main/crazyflie_esp_main.c`
- `espcontrol/crazyflie_esp/main/amg8833.c`
- `espcontrol/crazyflie_esp/main/uart_cf_comm.c`

### 3. Python Dashboard and Tools
Location:
- `tools/`

Main purpose:
- Connect to the Crazyflie via Crazyradio
- Start / stop the mission
- Display mission, range, and flow telemetry
- Log telemetry to CSV
- Export 2D and 3D path visualizations
- Open exported 3D path in a browser
- Provide thermal debugging tools

Important files:
- `tools/radio_wall_follow_dashboard.py`
- `tools/radio_wall_follow_mission.py`
- `tools/amg8833_heatmap_viewer.py`
- `tools/telemetry_logs/`

---
## Dashboard Requirements

## Hardware
- Crazyflie 2.1
- Crazyradio PA
- ESP32-C3 SuperMini
- AMG8833 thermal sensor
- Working UART connection between ESP32 and Crazyflie
- Required Crazyflie decks for your mission setup

## Software
- Python 3.10 or newer
- `tkinter`
- `cflib`
- `Pillow`
- `plotly`
- `playsound` (optional; macOS can also use `afplay`)
- Crazyradio USB driver / access permissions working on the host machine

---

## Recommended Python Environment for Dashboard

This repository already uses a dedicated dashboard environment:

- `.venv-radio`

You should use that environment to run the dashboard.

---

## Dashboard Installation

### 1. Create the dashboard virtual environment
If you need to create it from scratch:

```bash
cd /Users/zhangzehua/Desktop/fyp
python3 -m venv .venv-radio
source .venv-radio/bin/activate
pip install -r requirements.txt
```

## Repository Structure

```text
fyp/
├── crazyflie/
│   └── crazyflie-firmware/
│       ├── examples/demos/app_wall_following_demo/src/
│       │   └── wall_following.c
│       └── src/
│           ├── drivers/src/uart2.c
│           └── modules/src/esp_uart_bridge.c
│
├── espcontrol/
│   └── crazyflie_esp/
│       ├── main/
│       │   ├── crazyflie_esp_main.c
│       │   ├── amg8833.c
│       │   └── uart_cf_comm.c
│       └── build/
│
├── tools/
│   ├── radio_wall_follow_dashboard.py
│   ├── radio_wall_follow_mission.py
│   ├── amg8833_heatmap_viewer.py
│   └── telemetry_logs/
│
├── output/
├── REPORT/
└── README.md
