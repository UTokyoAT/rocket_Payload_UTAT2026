"""
CanSat 地上局レシーバー（推測航法テスト専用 / Windowsネイティブ / Tkinter）

XIAO ESP32S3 の SoftAP (CanSat-AP) に接続した状態で実行する。

tests/test_dead_reckoning・test_dead_reckoning_post_detach 専用の受信スクリプト。
これらのテストファームウェアはGPSを使わないため、SpiFrameToXiao2のlat/lonフィールドを
本来のGPS度数ではなく「着地位置（原点）からのx/y距離[m]」として、yawフィールドを
真北基準ではなく「原点リセット時の機体の向きを0度とする相対ヘディング[deg]」として送ってくる
（詳細は tests/test_dead_reckoning/main.cpp の「走行軌跡の推定」コメントを参照）。

本番用の receiver.py はGPS度数（緯度経度）を前提に地図を描画するため、このテストの
データをそのまま渡すと壊れた地図になる。本スクリプトはその変換をせず、受信した
lat/lonをそのままメートル単位のローカルXY座標として描画する版。

本番のGPS目的地（--dest-lat/--dest-lon, GET /goal）機能はこのテストでは意味がないため
持たない。CSVログ・ターミナル表示・地図表示・手動モーター制御（安全停止用）は
receiver.pyと同じ構成。

使い方:
    pip install -r requirements.txt   # GUIのマップ描画に matplotlib を使用
    python receiver_dead_reckoning.py                  # GUIあり
    python receiver_dead_reckoning.py --headless        # ターミナル＋CSVのみ
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

# ─── バイナリフレームレイアウト（機体側 include/spi_protocol.h の SpiFrameToXiao2 と共有する契約）───
# 37バイト、リトルエンディアン。フィールド構成は本番と同じだが、このテストでは
# lat/lonが「原点からのx/y距離[m]」、yawが「相対ヘディング[deg]」を表す（GPS/地磁気非使用）。
FRAME_FMT = "<IffffffBff"
FRAME_SIZE = struct.calcsize(FRAME_FMT)  # 37

Frame = namedtuple("Frame", ["t", "alt", "roll", "pitch", "heading", "x", "y", "state",
                              "pid_output", "phase_code"])

STATE_NAMES = {
    0: "SETTING",
    1: "LAUNCH",
    2: "DETACH",
    3: "UNFOLD",
    4: "NAVIGATE",
    5: "GOAL",
    6: "ABORTED",
}

# mission_state（STATE_NAMES）はXIAO2の安全ゲート用でDRIVE_STRAIGHT/DRIVE_CIRCLEどちらも
# NAVIGATE固定になり区別できないため、機体側はdestination_yawフィールドにこのテスト独自の
# 詳細フェーズ番号を載せて送ってくる（tests/test_dead_reckoning・test_dead_reckoning_post_detach
# 両方のmain.cppでこの番号に揃えてある）。
PHASE_NAMES = {
    0: "LAUNCH",
    1: "DETACH",
    2: "UNFOLD",           # post_detach版ではWAIT_ATTITUDE_SETTLE
    3: "DRIVE_STRAIGHT",
    4: "DRIVE_CIRCLE",
    5: "GOAL",              # post_detach版ではDONE
    6: "ABORTED",
}

CSV_HEADER = ["timestamp_ms", "alt_m", "roll_deg", "pitch_deg", "heading_deg",
              "x_m", "y_m", "state", "phase", "pid_output"]

# tests/test_dead_reckoning/main.cppのLAUNCH_ARM_DELAY_MS（起動からこの時間が経つまで
# 高度判定を開始しない）と同じ値。LAUNCH中の残り時間表示にだけ使う
# （post_detach版はLAUNCH状態自体が無いので単に表示されない）。
LAUNCH_ARM_DELAY_MS = 10 * 60 * 1000

LOG_DIR = Path(__file__).parent / "logs"


def format_elapsed(ms: int) -> str:
    """起動からの経過時間[ms]を H:MM:SS 形式にする。"""
    total_s = ms // 1000
    h, rem = divmod(total_s, 3600)
    m, s = divmod(rem, 60)
    return f"{h}:{m:02d}:{s:02d}"


def make_log_path() -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    return LOG_DIR / f"deadreckoning_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


def fmt(frame: Frame) -> str:
    state_name = STATE_NAMES.get(frame.state, str(frame.state))
    phase_code = int(round(frame.phase_code))
    phase_name = PHASE_NAMES.get(phase_code, str(phase_code))
    return (
        f"\r[{state_name:<9}/{phase_name:<14}] "
        f"T+{format_elapsed(frame.t):<8} "
        f"alt={frame.alt:7.2f}m  "
        f"R={frame.roll:6.1f}°  P={frame.pitch:6.1f}°  HDG={frame.heading:6.1f}°  "
        f"pos=({frame.x:6.2f},{frame.y:6.2f})m  "
        f"PID={frame.pid_output:6.1f}"
    )


class Poller(threading.Thread):
    """バックグラウンドスレッドで /data を定期的にGETし、結果をqueueへ流す。"""

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


def _send_motor_command(url: str, left: int, right: int, timeout_s: float) -> None:
    """/motor へのベストエフォート送信（失敗しても黙って諦める）。安全停止ボタン用。"""
    try:
        urllib.request.urlopen(f"{url}?left={left}&right={right}", timeout=timeout_s)
    except (urllib.error.URLError, OSError, TimeoutError):
        pass


class MotorCommander(threading.Thread):
    """有効化されている間、現在の左右値を /motor へ定期送信し続けるバックグラウンドスレッド
    （機体はNAVIGATE中は自律走行するが、緊急停止用に手動オーバーライドできるようにしておく）。
    """

    def __init__(self, url: str, send_interval_s: float, timeout_s: float,
                 stop_event: threading.Event):
        super().__init__(daemon=True)
        self.url = url
        self.send_interval_s = send_interval_s
        self.timeout_s = timeout_s
        self.stop_event = stop_event
        self.enabled = False
        self.value_left = 0
        self.value_right = 0

    def run(self) -> None:
        while not self.stop_event.is_set():
            if self.enabled:
                _send_motor_command(self.url, self.value_left, self.value_right, self.timeout_s)
            self.stop_event.wait(self.send_interval_s)


class Recorder:
    """CSV書き込み・ターミナル表示を担当する（GUI/headless共通）。"""

    def __init__(self):
        self.log_path = make_log_path()
        self._f = open(self.log_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._f)
        self._writer.writerow(CSV_HEADER)
        print(f"Logging to {self.log_path}\n")

    def on_data(self, frame: Frame) -> None:
        phase_code = int(round(frame.phase_code))
        self._writer.writerow([
            frame.t, frame.alt, frame.roll, frame.pitch, frame.heading,
            frame.x, frame.y, STATE_NAMES.get(frame.state, frame.state),
            PHASE_NAMES.get(phase_code, phase_code),
            frame.pid_output,
        ])
        self._f.flush()
        print(fmt(frame), end="", flush=True)

    def on_error(self, message: str) -> None:
        print(f"\rGET failed: {message}. retrying...", end="", flush=True)

    def close(self) -> None:
        self._f.close()


def run_headless(poller: Poller, recorder: Recorder,
                  out_queue: "queue.Queue", stop_event: threading.Event) -> None:
    poller.start()
    try:
        while True:
            kind, payload = out_queue.get()
            if kind == "data":
                recorder.on_data(payload)
            else:
                recorder.on_error(payload)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        poller.join(timeout=2)
        recorder.close()


def run_gui(poller: Poller, recorder: Recorder, motor_cmd: MotorCommander,
            out_queue: "queue.Queue", stop_event: threading.Event) -> None:
    import tkinter as tk

    import matplotlib
    # matplotlibの既定フォント(DejaVu Sans)には日本語グリフが無く、軸ラベルの日本語部分が
    # 豆腐（□）化するため、Windows標準の日本語フォントを明示指定する（Tkinter側のラベルは
    # OSのフォントフォールバックで問題なく表示されるので、matplotlib側だけの対応でよい）。
    matplotlib.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    matplotlib.rcParams["axes.unicode_minus"] = False  # 上記フォントだとマイナス記号が文字化けすることがあるため

    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    MAX_PTS = 300
    MAX_TRAIL = 2000  # 直進10m+円1周ぶんを1本の軌跡で見たいので長めに持つ
    alt_buf: list[float] = []
    trail: list[tuple[float, float]] = []

    BG = "#111"
    CARD_BG = "#1e1e1e"
    ACCENT = "#4fc"
    MUTED = "#888"
    FONT = ("Consolas", 11)
    LABEL_FONT = ("Consolas", 9)

    root = tk.Tk()
    root.title("CanSat Dead-Reckoning Monitor")
    root.configure(bg=BG)
    root.geometry("1180x820")
    root.minsize(900, 640)

    status_var = tk.StringVar(value="● unreachable")
    state_var = tk.StringVar(value="--")
    phase_var = tk.StringVar(value="--")
    elapsed_var = tk.StringVar(value="--")
    launch_arm_var = tk.StringVar(value="")
    alt_var = tk.StringVar(value="--")
    rph_var = tk.StringVar(value="--")
    pos_var = tk.StringVar(value="--")
    pid_var = tk.StringVar(value="--")

    root.columnconfigure(0, weight=0)
    root.columnconfigure(1, weight=1)
    root.rowconfigure(0, weight=1)

    left = tk.Frame(root, bg=BG)
    left.grid(row=0, column=0, sticky="ns")
    right = tk.Frame(root, bg=BG)
    right.grid(row=0, column=1, sticky="nsew")
    right.rowconfigure(0, weight=1)
    right.rowconfigure(1, weight=2)
    right.columnconfigure(0, weight=3)  # マップ（広め）
    right.columnconfigure(1, weight=1)  # 手動モーター制御（右下）

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

    card(left, "STATE（mission_state。安全ゲート用）", state_var, value_font=("Consolas", 14, "bold"))
    card(left, "PHASE（テスト詳細フェーズ）", phase_var, value_font=("Consolas", 16, "bold"))
    card(left, "起動からの経過時間 (T+)", elapsed_var, value_font=("Consolas", 16, "bold"))
    tk.Label(left, textvariable=launch_arm_var, bg=BG, fg="#fc4",
             font=LABEL_FONT, justify="left").pack(anchor="w", padx=12, pady=(0, 4))
    card(left, "ALTITUDE [m]", alt_var)
    card(left, "ROLL / PITCH / HEADING [deg]", rph_var, value_font=("Consolas", 13))
    card(left, "推測位置 (x, y) [m]", pos_var, value_font=("Consolas", 13))

    def make_motor_gauge(parent) -> tk.Canvas:
        gauge = tk.Canvas(parent, bg=BG, width=220, height=14, highlightthickness=0)
        gauge.pack(anchor="w", padx=10, pady=(0, 4))
        return gauge

    def draw_motor_gauge(gauge: tk.Canvas, speed: int):
        gauge.delete("all")
        w, h = 220, 14
        cx = w // 2
        gauge.create_line(cx, 0, cx, h, fill=MUTED)
        frac = max(-1.0, min(1.0, speed / 255.0))
        bw = frac * (w // 2 - 4)
        color = ACCENT if speed >= 0 else "#f84"
        gauge.create_rectangle(cx, 2, cx + bw, h - 2, fill=color, outline="")

    motor_card = tk.Frame(left, bg=CARD_BG)
    tk.Label(motor_card, text="PID OUTPUT (turn)", bg=CARD_BG, fg=MUTED,
             font=LABEL_FONT).pack(anchor="w", padx=10, pady=(8, 2))
    tk.Label(motor_card, textvariable=pid_var, bg=CARD_BG, fg=ACCENT,
              font=("Consolas", 14, "bold")).pack(anchor="w", padx=10)
    pid_gauge = make_motor_gauge(motor_card)
    motor_card.pack(fill="x", padx=12, pady=4)

    # ─── 手動モーター制御（緊急停止用。左右独立、画面右下に配置） ───
    manual_frame = tk.Frame(right, bg=CARD_BG)
    tk.Label(manual_frame, text="手動モーター制御（緊急停止用）", bg=CARD_BG, fg=MUTED,
             font=LABEL_FONT).pack(anchor="w", padx=10, pady=(8, 2))

    manual_enabled_var = tk.BooleanVar(value=False)
    manual_left_var = tk.IntVar(value=0)
    manual_right_var = tk.IntVar(value=0)

    def on_slider_left_change(val: str) -> None:
        motor_cmd.value_left = int(float(val))

    def on_slider_right_change(val: str) -> None:
        motor_cmd.value_right = int(float(val))

    def stop_both(send: bool = True) -> None:
        manual_left_var.set(0)
        manual_right_var.set(0)
        motor_cmd.value_left = 0
        motor_cmd.value_right = 0
        if send:
            threading.Thread(
                target=_send_motor_command,
                args=(motor_cmd.url, 0, 0, motor_cmd.timeout_s),
                daemon=True,
            ).start()

    def on_toggle_enable() -> None:
        enabled = manual_enabled_var.get()
        motor_cmd.enabled = enabled
        scale_left.configure(state=("normal" if enabled else "disabled"))
        scale_right.configure(state=("normal" if enabled else "disabled"))
        if not enabled:
            stop_both()

    tk.Checkbutton(manual_frame, text="有効にする\n（実際にモーターが動きます）",
                   variable=manual_enabled_var, command=on_toggle_enable,
                   bg=CARD_BG, fg="#eee", selectcolor=BG, activebackground=CARD_BG,
                   justify="left", font=LABEL_FONT).pack(anchor="w", padx=10)

    tk.Label(manual_frame, text="LEFT", bg=CARD_BG, fg=MUTED,
             font=LABEL_FONT).pack(anchor="w", padx=10, pady=(4, 0))
    scale_left = tk.Scale(manual_frame, from_=-255, to=255, orient="horizontal",
                           length=160, variable=manual_left_var, command=on_slider_left_change,
                           bg=CARD_BG, fg="#eee", troughcolor=BG, highlightthickness=0,
                           state="disabled")
    scale_left.pack(anchor="w", padx=10, fill="x")

    tk.Label(manual_frame, text="RIGHT", bg=CARD_BG, fg=MUTED,
             font=LABEL_FONT).pack(anchor="w", padx=10, pady=(4, 0))
    scale_right = tk.Scale(manual_frame, from_=-255, to=255, orient="horizontal",
                            length=160, variable=manual_right_var, command=on_slider_right_change,
                            bg=CARD_BG, fg="#eee", troughcolor=BG, highlightthickness=0,
                            state="disabled")
    scale_right.pack(anchor="w", padx=10, fill="x")

    tk.Button(manual_frame, text="STOP", command=stop_both,
              bg="#f44", fg="white", font=("Consolas", 10, "bold"),
              relief="flat").pack(anchor="w", padx=10, pady=(6, 10))

    manual_frame.grid(row=1, column=1, sticky="nsew", padx=(6, 12), pady=(6, 12))

    canvas = tk.Canvas(right, bg=BG, height=180, highlightthickness=0)
    canvas.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=12, pady=(12, 6))

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

    # ─── マップ（推測位置の軌跡。原点=着地位置、機体の向きは相対ヘディング）───
    fig = Figure(figsize=(5, 5), dpi=100, facecolor=BG)
    ax = fig.add_subplot(111)
    map_canvas = FigureCanvasTkAgg(fig, master=right)
    map_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=12, pady=(6, 12))

    def draw_map(frame: Frame):
        # 機体から送られてくる生のx_m/y_m・heading_degはそのままCSVに残す一方、
        # マップ描画だけは前後が逆に見えるとの実機報告があったためY軸を反転して表示する
        # （前進＝画面上方向になるようにするための表示専用の補正。ヘディング矢印も
        # 同じ反転をかけないと矢印だけ軌跡と逆向きになるので、dyも合わせて反転する）。
        x, y = frame.x, -frame.y
        trail.append((x, y))
        if len(trail) > MAX_TRAIL:
            trail.pop(0)

        ax.clear()
        ax.set_facecolor(BG)
        for spine in ax.spines.values():
            spine.set_color(MUTED)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(True, color=MUTED, alpha=0.15)

        xs = [p[0] for p in trail] + [0.0]
        ys = [p[1] for p in trail] + [0.0]
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
                     color=ACCENT, alpha=0.6, lw=1.5)

        # 原点＝着地位置
        ax.plot(0.0, 0.0, marker="*", color="#f44", markersize=14, zorder=5)
        ax.annotate("LANDING", (0.0, 0.0), color="#f44", fontsize=8,
                    xytext=(4, 4), textcoords="offset points")

        ax.plot(x, y, marker="o", color=ACCENT, markersize=9, zorder=6)
        arrow_len = span * 0.15
        heading_rad = math.radians(frame.heading)
        dx, dy = arrow_len * math.sin(heading_rad), -arrow_len * math.cos(heading_rad)
        ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle="-|>", color="#ff0", lw=2), zorder=7)

        ax.set_xlabel("X [m]（IMU推測・ドリフトあり）", color=MUTED, fontsize=8)
        ax.set_ylabel("Y [m]（IMU推測・ドリフトあり・表示用に符号反転）", color=MUTED, fontsize=8)
        map_canvas.draw_idle()

    def drain_queue():
        try:
            while True:
                kind, payload = out_queue.get_nowait()
                if kind == "data":
                    recorder.on_data(payload)

                    status_var.set("● reachable")
                    state_var.set(STATE_NAMES.get(payload.state, str(payload.state)))
                    phase_var.set(PHASE_NAMES.get(int(round(payload.phase_code)), str(payload.phase_code)))
                    elapsed_var.set(f"T+{format_elapsed(payload.t)}")
                    phase_code = int(round(payload.phase_code))
                    if phase_code == 0 and payload.t < LAUNCH_ARM_DELAY_MS:
                        remain_ms = LAUNCH_ARM_DELAY_MS - payload.t
                        launch_arm_var.set(f"高度判定 開始まで あと {format_elapsed(remain_ms)}")
                    else:
                        launch_arm_var.set("")
                    alt_var.set(f"{payload.alt:.1f}")
                    rph_var.set(f"R:{payload.roll:6.1f}  P:{payload.pitch:6.1f}  H:{payload.heading:6.1f}")
                    pos_var.set(f"({payload.x:.2f}, {payload.y:.2f})")
                    pid_var.set(f"{payload.pid_output:6.1f}")
                    draw_motor_gauge(pid_gauge, int(payload.pid_output))
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
        # 念のため終了時にもモーター停止を送っておく
        _send_motor_command(motor_cmd.url, 0, 0, motor_cmd.timeout_s)
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
    parser.add_argument("--headless", action="store_true",
                         help="Tkinter/matplotlibを使わずターミナル＋CSVのみで動作する")
    args = parser.parse_args()

    url = f"http://{args.host}:{args.port}{args.path}"
    motor_url = f"http://{args.host}:{args.port}/motor"
    print(f"Polling {url} every {args.interval_ms}ms ...")
    print("注意: lat/lonはGPS度数ではなく着地位置からのx/y距離[m]として解釈します"
          "（test_dead_reckoning系ファームウェア専用）。\n")

    out_queue: "queue.Queue" = queue.Queue()
    stop_event = threading.Event()
    poller = Poller(url, args.interval_ms / 1000, args.timeout, out_queue, stop_event)
    recorder = Recorder()

    if args.headless:
        run_headless(poller, recorder, out_queue, stop_event)
        return

    # 機体のフェイルセイフ（1秒無通信で自動的に出力0）より十分短い間隔で送り続ける
    motor_cmd = MotorCommander(motor_url, send_interval_s=0.25, timeout_s=0.5, stop_event=stop_event)

    try:
        run_gui(poller, recorder, motor_cmd, out_queue, stop_event)
    except ImportError as e:
        print(f"GUI依存関係が利用できないため --headless モードで動作します（{e}）。", file=sys.stderr)
        stop_event = threading.Event()
        poller = Poller(url, args.interval_ms / 1000, args.timeout, out_queue, stop_event)
        run_headless(poller, recorder, out_queue, stop_event)


if __name__ == "__main__":
    main()
