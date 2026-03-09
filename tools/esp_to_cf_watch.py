#!/usr/bin/env python3
"""Live terminal view of ESP32 -> Crazyflie UART reception.

Connects to Crazyflie over USB (default: usb://0), subscribes to:
  - espUart.testRxCount
  - espUart.testTxCount
  - espUart.cmdRxCount
  - stabilizer.thrust
  - ctrltarget.z
and streams ESPUART console lines.
"""

import argparse
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def main() -> int:
    parser = argparse.ArgumentParser(description="Watch ESP32->Crazyflie UART counters")
    parser.add_argument("--uri", default="usb://0", help="Crazyflie link URI (default: usb://0)")
    parser.add_argument("--duration", type=float, default=20.0, help="Watch time in seconds")
    args = parser.parse_args()

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")

    state = {
        "rx_first": None,
        "rx_last": None,
        "beep_first": None,
        "beep_last": None,
        "cmd_first": None,
        "cmd_last": None,
        "max_thrust": 0.0,
    }
    line_buf = []

    print(f"[watch] connecting to {args.uri}")

    def on_console(chars: str) -> None:
        for ch in chars:
            if ch == "\n":
                line = "".join(line_buf).strip("\r")
                line_buf.clear()
                if "ESPUART:" in line:
                    print(f"[console] {line}")
            else:
                line_buf.append(ch)

    def on_log(ts: int, data: dict, _logconf: LogConfig) -> None:
        rx = int(data["espUart.testRxCount"])
        tx = int(data["espUart.testTxCount"])
        beep = int(data["espUart.beepCount"])
        cmd = int(data["espUart.cmdRxCount"])
        thrust = float(data["stabilizer.thrust"])
        target_z = float(data["ctrltarget.z"])
        if state["rx_first"] is None:
            state["rx_first"] = rx
        if state["beep_first"] is None:
            state["beep_first"] = beep
        if state["cmd_first"] is None:
            state["cmd_first"] = cmd
        prev = state["rx_last"]
        state["rx_last"] = rx
        state["beep_last"] = beep
        state["cmd_last"] = cmd
        if thrust > state["max_thrust"]:
            state["max_thrust"] = thrust
        delta = 0 if prev is None else (rx - prev)
        print(
            f"[log] t={ts} rx={rx} (delta={delta:+d}) cmdRx={cmd} "
            f"beep={beep} tx={tx} thrust={thrust:.1f} targetZ={target_z:.2f}"
        )

    with SyncCrazyflie(args.uri, cf=cf):
        cf.console.receivedChar.add_callback(on_console)

        logconf = LogConfig(name="esp_uart_watch", period_in_ms=1000)
        logconf.add_variable("espUart.testRxCount", "uint32_t")
        logconf.add_variable("espUart.beepCount", "uint32_t")
        logconf.add_variable("espUart.testTxCount", "uint32_t")
        logconf.add_variable("espUart.cmdRxCount", "uint32_t")
        logconf.add_variable("stabilizer.thrust", "float")
        logconf.add_variable("ctrltarget.z", "float")
        logconf.data_received_cb.add_callback(on_log)
        cf.log.add_config(logconf)
        logconf.start()

        end_t = time.time() + args.duration
        while time.time() < end_t:
            time.sleep(0.1)

        logconf.stop()

    if state["rx_first"] is None or state["rx_last"] is None:
        print("[result] no espUart log samples received")
        return 1

    gained = state["rx_last"] - state["rx_first"]
    beep_gained = 0 if (state["beep_first"] is None or state["beep_last"] is None) else (state["beep_last"] - state["beep_first"])
    cmd_gained = 0 if (state["cmd_first"] is None or state["cmd_last"] is None) else (state["cmd_last"] - state["cmd_first"])
    if gained > 0:
        print(
            f"[result] PASS: espUart.testRxCount +{gained}, espUart.beepCount +{beep_gained}, "
            f"espUart.cmdRxCount +{cmd_gained}, maxThrust={state['max_thrust']:.1f}"
        )
        return 0

    print("[result] FAIL: espUart.testRxCount did not increase")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
