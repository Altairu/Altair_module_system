#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MDD（モータードライブ）制御 GUI ツール（Ubuntu用 socketcan対応）
状態連動 CAN プロトコル対応:
    - パラメータ設定中: 0x200-0x203 + 0x210(4B mode)
    - 制御実行中: 0x210(8B target)
    - ステータス返信: 0x220
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import queue
from datetime import datetime

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

# ───────────────────────────────────────────────
# 定数
# ───────────────────────────────────────────────
CAN_ID_PARAM_BASE = 0x200  # Motor1-4: 0x200-0x203
CAN_ID_MODE       = 0x210  # 4B モード設定
CAN_ID_TARGET     = 0x210  # 8B 目標値設定
CAN_ID_STATUS     = 0x220  # 6B ステータス返信

TX_INTERVAL_10MS = 0.010

APP_MODE_PARAMETER = 0
APP_MODE_CONTROL = 1

# コントロールモード
CONTROL_SPEED = 0
CONTROL_ANGLE = 1
MODE_NAMES = {CONTROL_SPEED: "速度", CONTROL_ANGLE: "角度"}

ERR_INIT_TIMEOUT = 0x01
ERR_CAN_RX_TIMEOUT = 0x02
ERR_CAN_TX_FAIL = 0x04

# ───────────────────────────────────────────────
# CAN 通信バックエンド
# ───────────────────────────────────────────────
class CANBackend:
    def __init__(self, channel: str, bitrate: int, log_queue: queue.Queue):
        self.channel    = channel
        self.bitrate    = bitrate
        self.log_queue  = log_queue
        self.bus        = None
        self.running    = False
        self.lock       = threading.Lock()

        # モータごとの制御パラメータ
        self._motors = [
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
        ]
        self.tx_enabled = False
        self.param_send_requested = False
        self.remote_app_mode = APP_MODE_PARAMETER
        self.last_remote_app_mode = None
        self.last_status_time = 0.0

    # ── セッター ────────────────────────────────
    def set_target(self, motor_index: int, val: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._motors[motor_index]["target"] = max(-32767, min(32767, int(val)))

    def set_mode(self, motor_index: int, mode: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._motors[motor_index]["mode"] = mode

    def set_param(self, motor_index: int, p: float, i: float, d: float, wheel: float, direction: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._motors[motor_index]["p"] = p
                self._motors[motor_index]["i"] = i
                self._motors[motor_index]["d"] = d
                self._motors[motor_index]["wheel"] = wheel
                self._motors[motor_index]["dir"] = 1 if direction >= 0 else -1

    # ── ゲッター ────────────────────────────────
    def get_motor(self, index: int):
        with self.lock:
            return dict(self._motors[index])

    # ── 接続（socketcan 優先） ──────────────────
    def _build_can_bus(self) -> bool:
        """socketcan (Linux native) を優先、それ以外は汎用インターフェース"""
        if not CAN_AVAILABLE:
            self._log("❌ python-can がインストールされていません → pip install python-can")
            return False
        
        try:
            # Ubuntu/Linux: socketcan (vcan0 など)
            self.bus = can.interface.Bus(interface='socketcan', channel=self.channel, bitrate=self.bitrate)
            self._log(f"✅ socketcan で接続: {self.channel} @ {self.bitrate} bps")
            return True
        except Exception as e1:
            self._log(f"⚠️ socketcan 失敗: {e1}")
            try:
                # フォールバック: 汎用インターフェース
                self.bus = can.interface.Bus(channel=self.channel, bitrate=self.bitrate)
                self._log(f"✅ 汎用インターフェースで接続: {self.channel} @ {self.bitrate} bps")
                return True
            except Exception as e2:
                self._log(f"❌ CAN 接続失敗: {e2}")
                return False

    def connect(self) -> bool:
        return self._build_can_bus()

    def disconnect(self):
        self.running = False
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self._log("🔌 切断しました")

    # ── スレッド起動 ─────────────────────────────
    def start(self):
        self.running = True
        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def request_parameter_send(self):
        self.param_send_requested = True
        self._log("📡 パラメータ設定送信を開始します")

    # ── 状態連動 10ms 送信ループ ─────────────────
    def _tx_loop(self):
        next_time_target = time.perf_counter()

        while self.running:
            now = time.perf_counter()

            if now >= next_time_target:
                if self.bus:
                    try:
                        self._send_by_remote_state()
                    except Exception as e:
                        self._log(f"⚠️ 送信エラー: {e}")
                next_time_target += TX_INTERVAL_10MS

            time.sleep(0.001)

    def _send_by_remote_state(self):
        if self.remote_app_mode == APP_MODE_PARAMETER:
            if self.param_send_requested:
                self._send_parameter_sequence()
        elif self.remote_app_mode == APP_MODE_CONTROL:
            if self.tx_enabled:
                self._send_target_frame()

    def _send_parameter_sequence(self):
        for motor_index in range(4):
            self._send_param_frame(motor_index)
        self._send_mode_frame()

    @staticmethod
    def _encode_i16_le(value: int):
        value &= 0xFFFF
        return value & 0xFF, (value >> 8) & 0xFF

    def _send_mode_frame(self):
        payload = bytearray(4)

        with self.lock:
            for i in range(4):
                payload[i] = self._motors[i]["mode"] & 0xFF

        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_MODE, data=payload)
        self.bus.send(msg)
        self._log(f"📤 0x210(mode) 送信: mode={[MODE_NAMES[self._motors[i]['mode']] for i in range(4)]}")

    def _send_target_frame(self):
        """0x210: 制御実行モード中の目標値 x10"""
        payload = bytearray(8)

        with self.lock:
            for i in range(4):
                scaled_target = int(round(float(self._motors[i]["target"]) * 10.0))
                scaled_target = max(-32768, min(32767, scaled_target))
                low, high = self._encode_i16_le(scaled_target)
                payload[i * 2] = low
                payload[i * 2 + 1] = high

        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_TARGET, data=payload[:8])
        self.bus.send(msg)
        self._log(f"📤 0x210(target) 送信: target={[self._motors[i]['target'] for i in range(4)]}")

    def _send_param_frame(self, motor_index: int):
        """0x200-0x203: 各モータ 8B パラメータ"""
        payload = bytearray(8)

        with self.lock:
            motor = self._motors[motor_index]
            p_raw = int(round(float(motor["p"]) * 100.0))
            i_raw = int(round(float(motor["i"]) * 100.0))
            d_raw = int(round(float(motor["d"]) * 100.0))
            wheel_raw = int(round(float(motor["wheel"])))
            direction = 1 if int(motor["dir"]) >= 0 else -1

        signed_wheel = wheel_raw * direction

        p_low, p_high = self._encode_i16_le(p_raw)
        i_low, i_high = self._encode_i16_le(i_raw)
        d_low, d_high = self._encode_i16_le(d_raw)
        w_low, w_high = self._encode_i16_le(signed_wheel)

        payload[0] = p_low
        payload[1] = p_high
        payload[2] = i_low
        payload[3] = i_high
        payload[4] = d_low
        payload[5] = d_high
        payload[6] = w_low
        payload[7] = w_high

        msg = can.Message(is_extended_id=False,
                          arbitration_id=CAN_ID_PARAM_BASE + motor_index,
                          data=payload)
        self.bus.send(msg)

        motor_data = self.get_motor(motor_index)
        self._log(
            f"📤 0x{CAN_ID_PARAM_BASE + motor_index:03X} 送信 (M{motor_index + 1}): "
            f"P={motor_data['p']:.2f}, I={motor_data['i']:.2f}, D={motor_data['d']:.2f}, "
            f"wheel={motor_data['wheel']:.1f}mm, dir={motor_data['dir']}"
        )

    def _rx_loop(self):
        """ステータス受信ループ"""
        while self.running:
            if self.bus:
                try:
                    msg = self.bus.recv(timeout=0.1)
                    if msg and msg.arbitration_id == CAN_ID_STATUS:
                        self._parse_status(msg.data)
                except Exception:
                    pass
            time.sleep(0.01)

    def _parse_status(self, data):
        """0x220: 4x Limit SW + Error + AppMode"""
        if len(data) < 6:
            return

        limit_states = [1 if data[i] else 0 for i in range(4)]
        error_code = data[4]
        app_mode = data[5] & 0x01
        previous_mode = self.remote_app_mode if self.last_remote_app_mode is not None else None

        self.last_remote_app_mode = self.remote_app_mode
        self.remote_app_mode = app_mode
        self.last_status_time = time.perf_counter()

        if previous_mode == APP_MODE_PARAMETER and app_mode == APP_MODE_CONTROL:
            self.param_send_requested = False
            self._log("✅ MDD が制御実行モードへ移行しました。パラメータ送信を停止します")

        if previous_mode == APP_MODE_CONTROL and app_mode == APP_MODE_PARAMETER:
            self.param_send_requested = False
            self.tx_enabled = False
            self._log("⛔ MDD が制御実行からパラメータ設定へ戻りました。送信を停止しました")

        limit_str = ", ".join([f"SW{i + 1}={'ON' if state else 'OFF'}" for i, state in enumerate(limit_states)])
        sys_str = "制御実行" if app_mode == APP_MODE_CONTROL else "パラメータ設定"
        err_flags = []
        if error_code & ERR_INIT_TIMEOUT:
            err_flags.append("INIT_TIMEOUT")
        if error_code & ERR_CAN_RX_TIMEOUT:
            err_flags.append("CAN_RX_TIMEOUT")
        if error_code & ERR_CAN_TX_FAIL:
            err_flags.append("CAN_TX_FAIL")
        err_str = "NONE" if not err_flags else ",".join(err_flags)
        self._log(f"📥 0x220: {limit_str} | State={sys_str} | Err=0x{error_code:02X}({err_str})")

    def _log(self, msg: str):
        try:
            self.log_queue.put_nowait(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
        except queue.Full:
            pass


# ───────────────────────────────────────────────
# GUI アプリケーション
# ───────────────────────────────────────────────
class AltairGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MDD 制御 GUI (Ubuntu socketcan)")
        self.root.geometry("1000x700")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        
        self.log_queue = queue.Queue(maxsize=100)
        self.backend = None
        self.var_remote_state = tk.StringVar(value="未接続")
        
        self._build_ui()
        self._update_log()

    def _build_ui(self):
        # ─── 接続パネル ─────────────────────────
        conn_frame = ttk.LabelFrame(self.root, text="CAN 接続 (socketcan)", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Channel (vcan0 など):").grid(row=0, column=0, sticky=tk.W)
        self.var_channel = tk.StringVar(value="vcan0")
        ttk.Entry(conn_frame, textvariable=self.var_channel).grid(row=0, column=1, sticky=tk.EW)
        
        ttk.Label(conn_frame, text="Bitrate (bps):").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.var_bitrate = tk.StringVar(value="1000000")
        ttk.Entry(conn_frame, textvariable=self.var_bitrate, width=10).grid(row=0, column=3)
        
        self.btn_connect = ttk.Button(conn_frame, text="接続", command=self._on_connect)
        self.btn_connect.grid(row=0, column=4, padx=10)
        
        self.btn_disconnect = ttk.Button(conn_frame, text="切断", command=self._on_disconnect, state=tk.DISABLED)
        self.btn_disconnect.grid(row=0, column=5)
        
        conn_frame.columnconfigure(1, weight=1)

        # ─── モータ制御パネル ────────────────────
        motors_frame = ttk.LabelFrame(self.root, text="モータ制御 (制御実行モード時のみ 10ms送信)", padding=10)
        motors_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.motor_vars = []
        for motor_idx in range(4):
            # フレーム
            m_frame = ttk.Frame(motors_frame)
            m_frame.pack(fill=tk.X, pady=5)
            
            ttk.Label(m_frame, text=f"M{motor_idx}", width=3, font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            
            # 目標値
            ttk.Label(m_frame, text="目標:").pack(side=tk.LEFT, padx=5)
            var_target = tk.DoubleVar(value=0.0)
            s_target = ttk.Scale(m_frame, from_=-32767, to=32767, variable=var_target,
                                orient=tk.HORIZONTAL, command=self._on_motor_change)
            s_target.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            e_target = ttk.Entry(m_frame, textvariable=var_target, width=8)
            e_target.pack(side=tk.LEFT, padx=5)
            
            # モード
            ttk.Label(m_frame, text="モード:").pack(side=tk.LEFT, padx=5)
            var_mode = tk.IntVar(value=CONTROL_SPEED)
            for mode_val, mode_name in MODE_NAMES.items():
                ttk.Radiobutton(m_frame, text=mode_name, variable=var_mode, value=mode_val,
                              command=self._on_motor_change).pack(side=tk.LEFT)
            
            self.motor_vars.append({
                "target": var_target,
                "mode": var_mode,
                "slider": s_target,
                "entry": e_target,
            })

        # ─── パラメータパネル ────────────────────
        param_frame = ttk.LabelFrame(self.root, text="パラメータ設定 (設定送信ボタン押下後 10ms送信)", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.param_vars = []
        for motor_idx in range(4):
            p_frame = ttk.Frame(param_frame)
            p_frame.pack(fill=tk.X, pady=3)
            
            ttk.Label(p_frame, text=f"M{motor_idx}", width=3).pack(side=tk.LEFT)
            ttk.Label(p_frame, text="P:").pack(side=tk.LEFT, padx=5)
            var_p = tk.StringVar(value="80")
            ttk.Entry(p_frame, textvariable=var_p, width=6).pack(side=tk.LEFT)
            
            ttk.Label(p_frame, text="I:").pack(side=tk.LEFT, padx=5)
            var_i = tk.StringVar(value="0")
            ttk.Entry(p_frame, textvariable=var_i, width=6).pack(side=tk.LEFT)
            
            ttk.Label(p_frame, text="D:").pack(side=tk.LEFT, padx=5)
            var_d = tk.StringVar(value="2")
            ttk.Entry(p_frame, textvariable=var_d, width=6).pack(side=tk.LEFT)
            
            ttk.Label(p_frame, text="Wheel(mm):").pack(side=tk.LEFT, padx=5)
            var_wheel = tk.StringVar(value="65")
            ttk.Entry(p_frame, textvariable=var_wheel, width=6).pack(side=tk.LEFT)

            ttk.Label(p_frame, text="Dir:").pack(side=tk.LEFT, padx=5)
            var_dir = tk.IntVar(value=1)
            ttk.Radiobutton(p_frame, text="+", variable=var_dir, value=1, command=self._on_motor_change).pack(side=tk.LEFT)
            ttk.Radiobutton(p_frame, text="-", variable=var_dir, value=-1, command=self._on_motor_change).pack(side=tk.LEFT)
            
            self.param_vars.append({
                "p": var_p, "i": var_i, "d": var_d, "wheel": var_wheel, "dir": var_dir,
            })

        # ─── TX 有効化 ───────────────────────
        ctrl_frame = ttk.Frame(self.root)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(ctrl_frame, text="MDD状態:").pack(side=tk.LEFT)
        ttk.Label(ctrl_frame, textvariable=self.var_remote_state).pack(side=tk.LEFT, padx=(4, 16))

        self.btn_param_send = ttk.Button(ctrl_frame, text="設定送信", command=self._on_start_param_send, state=tk.DISABLED)
        self.btn_param_send.pack(side=tk.LEFT)

        self.var_tx_enabled = tk.BooleanVar(value=False)
        self.cb_tx = ttk.Checkbutton(ctrl_frame, text="制御指令送信有効化", variable=self.var_tx_enabled,
                                    command=self._on_tx_toggle, state=tk.DISABLED)
        self.cb_tx.pack(side=tk.LEFT, padx=(16, 0))

        # ─── ログ ────────────────────────────
        log_frame = ttk.LabelFrame(self.root, text="ログ", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=120, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _on_connect(self):
        try:
            channel = self.var_channel.get()
            bitrate = int(self.var_bitrate.get())
            
            self.backend = CANBackend(channel, bitrate, self.log_queue)
            if self.backend.connect():
                self.backend.start()
                
                # UI 更新
                self.btn_connect.config(state=tk.DISABLED)
                self.btn_disconnect.config(state=tk.NORMAL)
                self.btn_param_send.config(state=tk.NORMAL)
                self.cb_tx.config(state=tk.NORMAL)
                for m in self.motor_vars:
                    m["slider"].config(state=tk.NORMAL)
                    m["entry"].config(state=tk.NORMAL)
            else:
                self.backend = None
        except ValueError:
            messagebox.showerror("エラー", "Bitrate は整数で指定してください")

    def _on_disconnect(self):
        if self.backend:
            self.backend.disconnect()
            self.backend = None
        
        # UI 更新
        self.btn_connect.config(state=tk.NORMAL)
        self.btn_disconnect.config(state=tk.DISABLED)
        self.btn_param_send.config(state=tk.DISABLED)
        self.cb_tx.config(state=tk.DISABLED)
        self.var_tx_enabled.set(False)
        self.var_remote_state.set("未接続")

    def _on_start_param_send(self):
        if self.backend:
            self._on_motor_change()
            self.backend.request_parameter_send()

    def _on_tx_toggle(self):
        if self.backend:
            self.backend.tx_enabled = self.var_tx_enabled.get()
            status = "有効" if self.backend.tx_enabled else "無効"
            self._log(f"📡 制御指令送信が {status} になりました")

    def _on_motor_change(self, *args):
        if self.backend:
            for idx, m_var in enumerate(self.motor_vars):
                try:
                    target = float(m_var["target"].get())
                except ValueError:
                    target = 0
                mode = m_var["mode"].get()
                
                self.backend.set_target(idx, target)
                self.backend.set_mode(idx, mode)
            
            # パラメータも同期
            for idx, p_var in enumerate(self.param_vars):
                try:
                    p = float(p_var["p"].get())
                    i = float(p_var["i"].get())
                    d = float(p_var["d"].get())
                    wheel = float(p_var["wheel"].get())
                    direction = int(p_var["dir"].get())
                    self.backend.set_param(idx, p, i, d, wheel, direction)
                except ValueError:
                    pass

    def _update_log(self):
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, msg + "\n")
                self.log_text.see(tk.END)
                self.log_text.config(state=tk.DISABLED)
        except queue.Empty:
            pass

        if self.backend:
            if self.backend.remote_app_mode == APP_MODE_CONTROL:
                self.var_remote_state.set("制御実行")
            else:
                self.var_remote_state.set("パラメータ設定")

            if self.var_tx_enabled.get() != self.backend.tx_enabled:
                self.var_tx_enabled.set(self.backend.tx_enabled)
        else:
            self.var_remote_state.set("未接続")
        
        self.root.after(100, self._update_log)

    def _log(self, msg: str):
        try:
            self.log_queue.put_nowait(msg)
        except queue.Full:
            pass

    def _on_close(self):
        if self.backend:
            self.backend.disconnect()
            self.backend = None
        self.root.destroy()


# ───────────────────────────────────────────────
# メイン
# ───────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = AltairGUI(root)
    root.mainloop()
