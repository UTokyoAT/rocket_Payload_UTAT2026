"""
XIAO ESP32S3 実機なしで receiver.py を動作確認するためのモック。
127.0.0.1 上で GET /data に対し、SensorData を模した33バイトフレームを返し続ける。
GET /motor?left=N&right=M も受け付け、lib/Radio/Radio.h と同じ1秒フェイルセイフで
motor_output_left/right に折り返す（GUIのスライダーの動作確認用）。

使い方:
    python mock_device.py
    # 別ターミナルで:
    python receiver.py --host 127.0.0.1 --port 8000
"""

import math
import struct
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# ground/receiver.py, lib/Radio/Radio.h と同じレイアウト・フォーマット文字列
FRAME_FMT = "<IffffffBhh"
FRAME_SIZE = struct.calcsize(FRAME_FMT)

# lib/Radio/Radio.h の MOTOR_COMMAND_TIMEOUT_MS と同じ
MOTOR_COMMAND_TIMEOUT_S = 1.0

HOST = "127.0.0.1"
PORT = 8000

_start = time.monotonic()
_motor_command_left = 0
_motor_command_right = 0
_motor_command_at = 0.0


def current_motor_output() -> tuple[int, int]:
    if time.monotonic() - _motor_command_at > MOTOR_COMMAND_TIMEOUT_S:
        return 0, 0
    return _motor_command_left, _motor_command_right


def current_frame() -> bytes:
    t = time.monotonic() - _start
    timestamp_ms = int(t * 1000) & 0xFFFFFFFF
    alt = 50.0 + 45.0 * math.sin(t * 0.4)
    roll = 25.0 * math.sin(t * 1.1)
    pitch = 12.0 * math.cos(t * 0.9)
    yaw = (t * 36.0) % 360.0
    lat = 35.681236 + 0.0002 * math.sin(t * 0.2)
    lon = 139.767125 + 0.0002 * math.cos(t * 0.2)
    state = int(t) // 10 % 6
    motor_l, motor_r = current_motor_output()
    return struct.pack(FRAME_FMT, timestamp_ms, alt, roll, pitch, yaw, lat, lon, state,
                        motor_l, motor_r)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # サーバーログを静かにする
        pass

    def do_GET(self):
        global _motor_command_left, _motor_command_right, _motor_command_at

        parsed = urlparse(self.path)
        if parsed.path == "/data":
            frame = current_frame()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return

        if parsed.path == "/motor":
            qs = parse_qs(parsed.query)
            if "left" not in qs or "right" not in qs:
                self.send_response(400)
                self.end_headers()
                return
            left = max(-255, min(255, int(qs["left"][0])))
            right = max(-255, min(255, int(qs["right"][0])))
            _motor_command_left = left
            _motor_command_right = right
            _motor_command_at = time.monotonic()
            print(f"[mock_device] motor command left={left} right={right}")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"ok")
            return

        self.send_response(404)
        self.end_headers()


def main() -> None:
    server = HTTPServer((HOST, PORT), Handler)
    print(f"[mock_device] serving fake {FRAME_SIZE}-byte frames at http://{HOST}:{PORT}/data")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
