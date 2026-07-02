"""
CanSat 地上局レシーバー（Windowsネイティブ / Tkinter）
XIAO ESP32S3 の SoftAP (CanSat-AP) に接続した状態で実行する。

PULL方式: このスクリプトが機体の GET /data を定期的にポーリングしにいく。
（機体→PCへのPUSH/POSTはPC側ファイアウォールに阻まれ信頼できないため使わない）

受信データを CSV に保存しつつ、ターミナルへのライブ表示と
Tkinter ウィンドウ（高度チャート・現在地/向き/目的地マップ・モーター出力・
目的地までの距離と方位・手動モーター制御スライダー）を表示する。

手動モーター制御は「有効にする」チェックを入れるとスライダーの値を
GET /motor?value=N として機体へ送り続ける（250ms間隔）。機体側は1秒間
コマンドを受信しないと自動的に出力を0にするフェイルセイフを持つ。

使い方:
    pip install -r requirements.txt   # GUIのマップ描画に matplotlib を使用
    python receiver.py                                    # GUIあり（目的地なし）
    python receiver.py --dest-lat 35.6820 --dest-lon 139.7670   # 目的地を指定
    python receiver.py --headless     # ターミナル＋CSVのみ（Tkinter/matplotlibなし環境向け）
"""

from __future__ import annotations

import argparse
import csv
import math
import queue
import struct
import sys
import threading
import urllib.error
import urllib.request
from collections import namedtuple
from datetime import datetime
from pathlib import Path

# ─── バイナリフレームレイアウト（機体側 lib/Radio/Radio.h と共有する契約）───
# 31バイト、リトルエンディアン。
#   offset  size  type     field           note
#     0      4    uint32   timestamp_ms
#     4      4    float32  alt             [m]
#     8      4    float32  roll            [deg]
#    12      4    float32  pitch           [deg]
#    16      4    float32  yaw             [deg]
#    20      4    float32  lat             double→float32に縮小
#    24      4    float32  lon             double→float32に縮小
#    28      1    uint8    mission_state
#    29      2    int16    motor_output    Actuator::setMotor()相当値（-255〜255）
FRAME_FMT = "<IffffffBh"
FRAME_SIZE = struct.calcsize(FRAME_FMT)  # 31

Frame = namedtuple("Frame", ["t", "alt", "roll", "pitch", "yaw", "lat", "lon", "state", "motor"])

STATE_NAMES = {
    0: "STANDBY",
    1: "ASCENDING",
    2: "DESCENDING",
    3: "SEPARATING",
    4: "RUNNING",
    5: "GOAL",
}

CSV_HEADER = ["timestamp_ms", "alt_m", "roll_deg", "pitch_deg", "yaw_deg",
              "lat", "lon", "state", "motor_output", "dist_to_dest_m", "bearing_to_dest_deg"]

LOG_DIR = Path(__file__).parent / "logs"

EARTH_RADIUS_M = 6371000.0


def make_log_path() -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    return LOG_DIR / f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """lat1,lon1 から lat2,lon2 への初期方位角 [deg]（0=北、時計回り）"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    x = math.sin(dlmb) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlmb)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def local_xy_m(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """ref(lat,lon)を原点とした東西(x)・南北(y)方向の平面近似座標 [m]。短距離用の簡易投影。"""
    x = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    y = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    return x, y


def fmt(frame: Frame, dist_m: float | None, brg: float | None) -> str:
    state_name = STATE_NAMES.get(frame.state, str(frame.state))
    line = (
        f"\r[{state_name:<12}] "
        f"alt={frame.alt:7.2f}m  "
        f"R={frame.roll:6.1f}°  P={frame.pitch:6.1f}°  Y={frame.yaw:6.1f}°  "
        f"GPS={frame.lat:.5f},{frame.lon:.5f}  M={frame.motor:4d}"
    )
    if dist_m is not None:
        line += f"  DIST={dist_m:7.1f}m BRG={brg:5.1f}°"
    return line


class Poller(threading.Thread):
    """バックグラウンドスレッドで /data を定期的にGETし、結果をqueueへ流す。

    GUIのメインスレッドをネットワーク待ちでブロックしないためにスレッド分離している。
    """

    def __init__(self, url: str, interval_s: float, timeout_s: float,
                 out_queue: "queue.Queue", stop_event: threading.Event):
        super().__init__(daemon=True)
        self.url = url
        self.interval_s = interval_s
        self.timeout_s = timeout_s
        self.out_queue = out_queue
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout_s) as resp:
                    raw = resp.read()
                if len(raw) != FRAME_SIZE:
                    self.out_queue.put(("error", f"frame size mismatch: got {len(raw)}, need {FRAME_SIZE}"))
                else:
                    self.out_queue.put(("data", Frame(*struct.unpack(FRAME_FMT, raw))))
            except (urllib.error.URLError, OSError, TimeoutError) as e:
                self.out_queue.put(("error", str(e)))
            self.stop_event.wait(self.interval_s)


def _send_motor_command(url: str, value: int, timeout_s: float) -> None:
    """/motor へのベストエフォート送信（失敗しても黙って諦める。呼び出し側でリトライする前提）。"""
    try:
        urllib.request.urlopen(f"{url}?value={value}", timeout=timeout_s)
    except (urllib.error.URLError, OSError, TimeoutError):
        pass


class MotorCommander(threading.Thread):
    """有効化されている間、現在値を /motor へ定期送信し続けるバックグラウンドスレッド。

    機体側 (Radio::getMotorCommand()) は一定時間コマンドを受信しないと自動的に
    出力を0にするフェイルセイフを持つため、ここでは「有効」な間は常に再送し続ける
    （スライダーの値が変わっていなくても、接続維持のために送り続ける必要がある）。
    """

    def __init__(self, url: str, send_interval_s: float, timeout_s: float,
                 stop_event: threading.Event):
        super().__init__(daemon=True)
        self.url = url
        self.send_interval_s = send_interval_s
        self.timeout_s = timeout_s
        self.stop_event = stop_event
        self.enabled = False
        self.value = 0

    def run(self) -> None:
        while not self.stop_event.is_set():
            if self.enabled:
                _send_motor_command(self.url, self.value, self.timeout_s)
            self.stop_event.wait(self.send_interval_s)


class Recorder:
    """CSV書き込み・ターミナル表示を担当する（GUI/headless共通）。"""

    def __init__(self):
        self.log_path = make_log_path()
        self._f = open(self.log_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._f)
        self._writer.writerow(CSV_HEADER)
        print(f"Logging to {self.log_path}\n")

    def on_data(self, frame: Frame, dist_m: float | None, brg: float | None) -> None:
        self._writer.writerow([
            frame.t, frame.alt, frame.roll, frame.pitch, frame.yaw,
            frame.lat, frame.lon, STATE_NAMES.get(frame.state, frame.state),
            frame.motor,
            "" if dist_m is None else f"{dist_m:.2f}",
            "" if brg is None else f"{brg:.1f}",
        ])
        self._f.flush()
        print(fmt(frame, dist_m, brg), end="", flush=True)

    def on_error(self, message: str) -> None:
        print(f"\rGET failed: {message}. retrying...", end="", flush=True)

    def close(self) -> None:
        self._f.close()


class DestinationTracker:
    """目的地（任意）までの距離・方位を計算し、地図描画用のローカル座標原点を管理する。"""

    def __init__(self, dest_lat: float | None, dest_lon: float | None):
        self.dest = (dest_lat, dest_lon) if dest_lat is not None else None
        # 目的地があればそこを原点に、なければ最初に受信した位置を原点にする
        self.ref = self.dest

    def update(self, frame: Frame) -> tuple[float | None, float | None]:
        if self.ref is None:
            self.ref = (frame.lat, frame.lon)
        dist = brg = None
        if self.dest is not None:
            dist = haversine_m(frame.lat, frame.lon, *self.dest)
            brg = bearing_deg(frame.lat, frame.lon, *self.dest)
        return dist, brg

    def current_xy(self, frame: Frame) -> tuple[float, float]:
        return local_xy_m(frame.lat, frame.lon, *self.ref)

    def dest_xy(self) -> tuple[float, float] | None:
        if self.dest is None or self.ref is None:
            return None
        return local_xy_m(self.dest[0], self.dest[1], *self.ref)


def run_headless(poller: Poller, recorder: Recorder, dest: DestinationTracker,
                  out_queue: "queue.Queue", stop_event: threading.Event) -> None:
    poller.start()
    try:
        while True:
            kind, payload = out_queue.get()
            if kind == "data":
                dist, brg = dest.update(payload)
                recorder.on_data(payload, dist, brg)
            else:
                recorder.on_error(payload)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        poller.join(timeout=2)
        recorder.close()


def run_gui(poller: Poller, recorder: Recorder, dest: DestinationTracker,
            motor_cmd: MotorCommander,
            out_queue: "queue.Queue", stop_event: threading.Event) -> None:
    import tkinter as tk

    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    MAX_PTS = 300
    MAX_TRAIL = 300
    alt_buf: list[float] = []
    trail: list[tuple[float, float]] = []

    BG = "#111"
    CARD_BG = "#1e1e1e"
    ACCENT = "#4fc"
    MUTED = "#888"
    FONT = ("Consolas", 11)
    LABEL_FONT = ("Consolas", 9)

    root = tk.Tk()
    root.title("CanSat Monitor")
    root.configure(bg=BG)
    root.geometry("1180x820")
    root.minsize(900, 640)

    status_var = tk.StringVar(value="● unreachable")
    state_var = tk.StringVar(value="--")
    alt_var = tk.StringVar(value="--")
    rpy_var = tk.StringVar(value="--")
    gps_var = tk.StringVar(value="--")
    motor_var = tk.StringVar(value="--")
    dist_var = tk.StringVar(value="--" if dest.dest is None else "取得中...")

    root.columnconfigure(0, weight=0)
    root.columnconfigure(1, weight=1)
    root.rowconfigure(0, weight=1)

    left = tk.Frame(root, bg=BG)
    left.grid(row=0, column=0, sticky="ns")
    right = tk.Frame(root, bg=BG)
    right.grid(row=0, column=1, sticky="nsew")
    right.rowconfigure(0, weight=1)
    right.rowconfigure(1, weight=2)
    right.columnconfigure(0, weight=1)

    tk.Label(left, textvariable=status_var, bg=BG, fg="#f44",
             font=FONT).pack(anchor="w", padx=12, pady=(10, 4))

    def card(parent, label: str, var: tk.StringVar, value_font=("Consolas", 18, "bold")):
        frame = tk.Frame(parent, bg=CARD_BG)
        tk.Label(frame, text=label, bg=CARD_BG, fg=MUTED,
                 font=LABEL_FONT).pack(anchor="w", padx=10, pady=(8, 2))
        tk.Label(frame, textvariable=var, bg=CARD_BG, fg=ACCENT,
                 font=value_font).pack(anchor="w", padx=10, pady=(0, 8))
        frame.pack(fill="x", padx=12, pady=4)
        return frame

    card(left, "STATE", state_var)
    card(left, "ALTITUDE [m]", alt_var)
    card(left, "ROLL / PITCH / YAW [deg]", rpy_var, value_font=("Consolas", 13))
    card(left, "GPS (lat, lon)", gps_var, value_font=("Consolas", 13))

    motor_card = card(left, "MOTOR OUTPUT", motor_var)
    motor_gauge = tk.Canvas(motor_card, bg=BG, width=220, height=16, highlightthickness=0)
    motor_gauge.pack(anchor="w", padx=10, pady=(0, 10))

    def draw_motor_gauge(speed: int):
        motor_gauge.delete("all")
        w, h = 220, 16
        cx = w // 2
        motor_gauge.create_line(cx, 0, cx, h, fill=MUTED)
        frac = max(-1.0, min(1.0, speed / 255.0))
        bw = frac * (w // 2 - 4)
        color = ACCENT if speed >= 0 else "#f84"
        motor_gauge.create_rectangle(cx, 2, cx + bw, h - 2, fill=color, outline="")

    card(left, "目的地までの距離 / 方位", dist_var, value_font=("Consolas", 13))

    # ─── 手動モーター制御 ───
    manual_frame = tk.Frame(left, bg=CARD_BG)
    tk.Label(manual_frame, text="手動モーター制御", bg=CARD_BG, fg=MUTED,
             font=LABEL_FONT).pack(anchor="w", padx=10, pady=(8, 2))

    manual_enabled_var = tk.BooleanVar(value=False)
    manual_value_var = tk.IntVar(value=0)

    def on_slider_change(val: str) -> None:
        motor_cmd.value = int(float(val))

    def on_toggle_enable() -> None:
        enabled = manual_enabled_var.get()
        motor_cmd.enabled = enabled
        scale.configure(state=("normal" if enabled else "disabled"))
        if not enabled:
            manual_value_var.set(0)
            motor_cmd.value = 0
            threading.Thread(
                target=_send_motor_command,
                args=(motor_cmd.url, 0, motor_cmd.timeout_s),
                daemon=True,
            ).start()

    tk.Checkbutton(manual_frame, text="有効にする（機体のモーターが実際に動きます）",
                   variable=manual_enabled_var, command=on_toggle_enable,
                   bg=CARD_BG, fg="#eee", selectcolor=BG, activebackground=CARD_BG,
                   font=LABEL_FONT).pack(anchor="w", padx=10)

    scale = tk.Scale(manual_frame, from_=-255, to=255, orient="horizontal",
                      length=220, variable=manual_value_var, command=on_slider_change,
                      bg=CARD_BG, fg="#eee", troughcolor=BG, highlightthickness=0,
                      state="disabled")
    scale.pack(anchor="w", padx=10, pady=(4, 4))

    def on_stop_button() -> None:
        manual_value_var.set(0)
        motor_cmd.value = 0
        threading.Thread(
            target=_send_motor_command,
            args=(motor_cmd.url, 0, motor_cmd.timeout_s),
            daemon=True,
        ).start()

    tk.Button(manual_frame, text="STOP", command=on_stop_button,
              bg="#f44", fg="white", font=("Consolas", 10, "bold"),
              relief="flat").pack(anchor="w", padx=10, pady=(0, 10))

    manual_frame.pack(fill="x", padx=12, pady=4)

    canvas = tk.Canvas(right, bg=BG, height=180, highlightthickness=0)
    canvas.grid(row=0, column=0, sticky="nsew", padx=12, pady=(12, 6))

    def draw_chart():
        canvas.delete("all")
        w = canvas.winfo_width() or 600
        h = canvas.winfo_height() or 180
        pad = 10
        if len(alt_buf) < 2:
            return
        max_v = max(max(alt_buf), 10)
        min_v = min(min(alt_buf), 0)
        rng = (max_v - min_v) or 1
        pts = []
        for i, v in enumerate(alt_buf):
            x = (i / (MAX_PTS - 1)) * w
            y = h - pad - ((v - min_v) / rng) * (h - pad * 2)
            pts.extend([x, y])
        if len(pts) >= 4:
            canvas.create_line(*pts, fill=ACCENT, width=2)
        canvas.create_text(24, pad + 6, text=f"{max_v:.1f}m", fill=ACCENT,
                            font=LABEL_FONT, anchor="w")
        canvas.create_text(24, h - 8, text=f"{min_v:.1f}m", fill=ACCENT,
                            font=LABEL_FONT, anchor="w")

    # ─── マップ（現在地・機体の向き・目的地・東西南北）───
    fig = Figure(figsize=(5, 5), dpi=100, facecolor=BG)
    ax = fig.add_subplot(111)
    map_canvas = FigureCanvasTkAgg(fig, master=right)
    map_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=12, pady=(6, 12))

    def draw_map(frame: Frame):
        x, y = dest.current_xy(frame)
        trail.append((x, y))
        if len(trail) > MAX_TRAIL:
            trail.pop(0)
        dxy = dest.dest_xy()

        ax.clear()
        ax.set_facecolor(BG)
        for spine in ax.spines.values():
            spine.set_color(MUTED)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(True, color=MUTED, alpha=0.15)

        xs = [p[0] for p in trail]
        ys = [p[1] for p in trail]
        if dxy is not None:
            xs.append(dxy[0])
            ys.append(dxy[1])
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        span = max(xmax - xmin, ymax - ymin, 10.0)
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        half = span / 2 * 1.3
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        ax.set_aspect("equal", adjustable="box")

        if len(trail) >= 2:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], "-",
                     color=ACCENT, alpha=0.4, lw=1.5)

        if dxy is not None:
            ax.plot(dxy[0], dxy[1], marker="*", color="#f44", markersize=16, zorder=5)
            ax.annotate("DEST", dxy, color="#f44", fontsize=8,
                        xytext=(4, 4), textcoords="offset points")

        ax.plot(x, y, marker="o", color=ACCENT, markersize=9, zorder=6)
        arrow_len = span * 0.15
        yaw_rad = math.radians(frame.yaw)
        dx, dy = arrow_len * math.sin(yaw_rad), arrow_len * math.cos(yaw_rad)
        ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle="-|>", color="#ff0", lw=2), zorder=7)

        # 東西南北（Y=北, X=東の平面近似なので軸の上下左右がそのままN/S/E/W）
        ax.text(0.5, 0.98, "N", transform=ax.transAxes, color=MUTED,
                ha="center", va="top", fontsize=10)
        ax.text(0.5, 0.02, "S", transform=ax.transAxes, color=MUTED,
                ha="center", va="bottom", fontsize=10)
        ax.text(0.98, 0.5, "E", transform=ax.transAxes, color=MUTED,
                ha="right", va="center", fontsize=10)
        ax.text(0.02, 0.5, "W", transform=ax.transAxes, color=MUTED,
                ha="left", va="center", fontsize=10)

        ax.set_xlabel("East [m]", color=MUTED, fontsize=8)
        ax.set_ylabel("North [m]", color=MUTED, fontsize=8)
        map_canvas.draw_idle()

    def drain_queue():
        try:
            while True:
                kind, payload = out_queue.get_nowait()
                if kind == "data":
                    dist, brg = dest.update(payload)
                    recorder.on_data(payload, dist, brg)

                    status_var.set("● reachable")
                    state_var.set(STATE_NAMES.get(payload.state, str(payload.state)))
                    alt_var.set(f"{payload.alt:.1f}")
                    rpy_var.set(f"R:{payload.roll:6.1f}  P:{payload.pitch:6.1f}  Y:{payload.yaw:6.1f}")
                    gps_var.set(f"{payload.lat:.6f}, {payload.lon:.6f}")
                    motor_var.set(str(payload.motor))
                    draw_motor_gauge(payload.motor)
                    if dist is not None:
                        dist_var.set(f"{dist:.1f} m  /  {brg:.1f}°")
                    alt_buf.append(payload.alt)
                    if len(alt_buf) > MAX_PTS:
                        alt_buf.pop(0)
                    draw_chart()
                    draw_map(payload)
                else:
                    recorder.on_error(payload)
                    status_var.set("● unreachable")
        except queue.Empty:
            pass
        root.after(100, drain_queue)

    def on_close():
        # 念のため終了時にもモーター停止を送っておく（機体側フェイルセイフの二重化）
        _send_motor_command(motor_cmd.url, 0, motor_cmd.timeout_s)
        stop_event.set()
        poller.join(timeout=2)
        motor_cmd.join(timeout=2)
        recorder.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<Configure>", lambda e: draw_chart())
    poller.start()
    motor_cmd.start()
    root.after(100, drain_queue)
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1", help="機体のSoftAP IP")
    parser.add_argument("--port", type=int, default=80)
    parser.add_argument("--path", default="/data")
    parser.add_argument("--interval-ms", type=int, default=120,
                         help="GETポーリング間隔 [ms]")
    parser.add_argument("--timeout", type=float, default=0.5,
                         help="GETタイムアウト [秒]")
    parser.add_argument("--dest-lat", type=float, default=None,
                         help="目的地の緯度（距離・方位・地図表示に使用。省略可）")
    parser.add_argument("--dest-lon", type=float, default=None,
                         help="目的地の経度（--dest-latとセットで指定する）")
    parser.add_argument("--headless", action="store_true",
                         help="Tkinter/matplotlibを使わずターミナル＋CSVのみで動作する")
    args = parser.parse_args()

    if (args.dest_lat is None) != (args.dest_lon is None):
        parser.error("--dest-lat と --dest-lon はセットで指定してください")

    url = f"http://{args.host}:{args.port}{args.path}"
    motor_url = f"http://{args.host}:{args.port}/motor"
    print(f"Polling {url} every {args.interval_ms}ms ...")
    if args.dest_lat is not None:
        print(f"Destination: {args.dest_lat:.6f}, {args.dest_lon:.6f}")

    out_queue: "queue.Queue" = queue.Queue()
    stop_event = threading.Event()
    poller = Poller(url, args.interval_ms / 1000, args.timeout, out_queue, stop_event)
    recorder = Recorder()
    dest = DestinationTracker(args.dest_lat, args.dest_lon)

    if args.headless:
        run_headless(poller, recorder, dest, out_queue, stop_event)
        return

    # 機体のフェイルセイフ（1秒無通信で自動的に出力0）より十分短い間隔で送り続ける
    motor_cmd = MotorCommander(motor_url, send_interval_s=0.25, timeout_s=0.5, stop_event=stop_event)

    try:
        run_gui(poller, recorder, dest, motor_cmd, out_queue, stop_event)
    except ImportError as e:
        print(f"GUI依存関係が利用できないため --headless モードで動作します（{e}）。", file=sys.stderr)
        stop_event = threading.Event()
        poller = Poller(url, args.interval_ms / 1000, args.timeout, out_queue, stop_event)
        run_headless(poller, recorder, dest, out_queue, stop_event)


if __name__ == "__main__":
    main()
