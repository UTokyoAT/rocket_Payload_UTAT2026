"""
CanSat 地上局レシーバー
ESP32 SoftAP (CanSat-AP) に接続した状態で実行する。
受信データを CSV に保存しながら、ターミナルにリアルタイム表示する。

使い方:
    pip install -r requirements.txt
    python receiver.py
"""

import asyncio
import websockets
import json
import csv
import sys
from datetime import datetime
from pathlib import Path

WS_URL = "ws://192.168.4.1/ws"
LOG_DIR = Path(__file__).parent / "logs"

STATE_NAMES = {
    0: "STANDBY",
    1: "ASCENDING",
    2: "FALLING",
    3: "SEPARATING",
    4: "RUNNING",
    5: "GOAL",
}

CSV_HEADER = ["timestamp_ms", "alt_m", "roll_deg", "pitch_deg",
              "yaw_deg", "lat", "lon", "state"]


def make_log_path() -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    return LOG_DIR / f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


def fmt(d: dict) -> str:
    state = STATE_NAMES.get(d["state"], str(d["state"]))
    return (
        f"\r[{state:<12}] "
        f"alt={d['alt']:7.2f}m  "
        f"R={d['roll']:6.1f}°  P={d['pitch']:6.1f}°  Y={d['yaw']:6.1f}°  "
        f"GPS={d['lat']:.5f},{d['lon']:.5f}"
    )


async def receive():
    log_path = make_log_path()
    print(f"Connecting to {WS_URL} ...")

    async with websockets.connect(WS_URL, ping_interval=5) as ws:
        print(f"Connected. Logging to {log_path}\n")

        with open(log_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)

            async for message in ws:
                d = json.loads(message)
                writer.writerow([
                    d["t"], d["alt"], d["roll"], d["pitch"],
                    d["yaw"], d["lat"], d["lon"],
                    STATE_NAMES.get(d["state"], d["state"]),
                ])
                f.flush()
                print(fmt(d), end="", flush=True)


async def main():
    while True:
        try:
            await receive()
        except (websockets.ConnectionClosed, OSError) as e:
            print(f"\nDisconnected ({e}). Reconnecting in 3s...")
            await asyncio.sleep(3)
        except KeyboardInterrupt:
            print("\nStopped.")
            sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
