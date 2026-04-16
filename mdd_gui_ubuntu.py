#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MDD（モータードライブ）制御 GUI ツール（Ubuntu用 socketcan対応）
新統合 CAN プロトコル対応:
  - 0x200: マルチプレクス パラメータ設定
  - 0x210: 統合 目標値 + モード指令 (MSB=モードフラグ)
  - 0x120: 統合 ステータス返信
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import struct
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
CAN_ID_PARAM   = 0x200  # パラメータ（マルチプレクス）
CAN_ID_TARGET  = 0x210  # 目標値 + モード統合
CAN_ID_STATUS  = 0x120  # ステータス返信

TX_INTERVAL_10MS  = 0.001   # 1ms 目標値送信周期
TX_INTERVAL_100MS = 0.010   # 10ms パラメータ送信周期

# コントロールモード
CONTROL_SPEED = 0
CONTROL_ANGLE = 1
MODE_NAMES = {CONTROL_SPEED: "速度", CONTROL_ANGLE: "角度"}

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
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65},
        ]
        self.tx_enabled = False
        self.last_param_tx_time = 0.0

    # ── セッター ────────────────────────────────
    def set_target(self, motor_index: int, val: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._motors[motor_index]["target"] = max(-32767, min(32767, int(val)))

    def set_mode(self, motor_index: int, mode: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._motors[motor_index]["mode"] = mode

    def set_param(self, motor_index: int, p: float, i: float, d: float, wheel: float):
        with self.lock:
            if 0 <= motor_index < 4:
                self._motors[motor_index]["p"] = p
                self._motors[motor_index]["i"] = i
                self._motors[motor_index]["d"] = d
                self._motors[motor_index]["wheel"] = wheel

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
        self.last_param_tx_time = time.perf_counter()
        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def stop(self):
        self.running = False

    # ── 10ms/100ms 送信ループ ───────────────────
    def _tx_loop(self):
        next_time_target = time.perf_counter()
        next_time_param = time.perf_counter() + 0.1  # 100ms後に最初のパラメータ送信
        
        while self.running:
            now = time.perf_counter()
            
            # 10ms: 目標値 + モード送信
            if now >= next_time_target:
                if self.bus and self.tx_enabled:
                    try:
                        self._send_target_frame()
                    except Exception as e:
                        self._log(f"⚠️ 目標値送信エラー: {e}")
                next_time_target += TX_INTERVAL_10MS
            
            # 100ms: パラメータ送信（ローテーション）
            if now >= next_time_param:
                if self.bus and self.tx_enabled:
                    try:
                        param_index = int((now - self.last_param_tx_time) / TX_INTERVAL_100MS) % 4
                        self._send_param_frame(param_index)
                    except Exception as e:
                        self._log(f"⚠️ パラメータ送信エラー: {e}")
                next_time_param += TX_INTERVAL_100MS
            
            time.sleep(0.001)

    def _send_target_frame(self):
        """0x210: 目標値(15bit) + MSB=モード"""
        payload = bytearray(8)
        
        with self.lock:
            for i in range(4):
                target = self._motors[i]["target"]
                mode = self._motors[i]["mode"]
                
                # MSB をモードで設定
                value = target & 0x7FFF
                if mode == CONTROL_ANGLE:
                    value |= 0x8000
                
                # リトルエンディアン
                payload[i*2] = value & 0xFF
                payload[i*2 + 1] = (value >> 8) & 0xFF
        
        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_TARGET, data=payload[:8])
        self.bus.send(msg)
        self._log(f"📤 0x210 送信: target={[self._motors[i]['target'] for i in range(4)]}, mode={[MODE_NAMES[self._motors[i]['mode']] for i in range(4)]}")

    def _send_param_frame(self, motor_index: int):
        """0x200: マルチプレクス パラメータ"""
        payload = bytearray(7)
        
        with self.lock:
            motor = self._motors[motor_index]
            
            # Byte 0: モータインデックス
            payload[0] = motor_index
            
            # Byte 1-3: P, I, D (int8 × 100 スケール)
            payload[1] = int(motor["p"] / 100) & 0xFF
            payload[2] = int(motor["i"] / 100) & 0xFF
            payload[3] = int(motor["d"] / 100) & 0xFF
            
            # Byte 4-5: 車輪径 (int16_t LE)
            wheel_val = int(motor["wheel"]) & 0xFFFF
            payload[4] = wheel_val & 0xFF
            payload[5] = (wheel_val >> 8) & 0xFF
            
            # Byte 6: 設定フラグ（将来用）
            payload[6] = 0x00
        
        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_PARAM, data=payload[:7])
        self.bus.send(msg)
        
        motor_data = self.get_motor(motor_index)
        self._log(f"📤 0x200 送信 (M{motor_index}): P={motor_data['p']:.2f}, I={motor_data['i']:.2f}, D={motor_data['d']:.2f}, wheel={motor_data['wheel']:.1f}mm")

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
        """0x120: 統合ステータス解析"""
        if len(data) < 5:
            return
        
        status_byte = data[0]
        limit_state = status_byte & 0x0F
        sys_state = (status_byte >> 4) & 0x01
        error_code = (status_byte >> 5) & 0x07
        
        modes = [data[i+1] & 0x01 for i in range(4)] if len(data) >= 5 else [0]*4
        
        limit_str = "".join([f"M{i}:{('ON' if limit_state & (1<<i) else 'OFF')}" for i in range(4)])
        sys_str = "制御中" if sys_state else "パラメータ待機中"
        mode_str = ",".join([MODE_NAMES.get(m, "?") for m in modes])
        
        self._log(f"📥 0x120: {limit_str} | SysState={sys_str} | Mode={mode_str} | Err={error_code:03b}")

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
        
        self.log_queue = queue.Queue(maxsize=100)
        self.backend = None
        
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
        motors_frame = ttk.LabelFrame(self.root, text="モータ制御 (10ms 周期送信)", padding=10)
        motors_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.motor_vars = []
        for motor_idx in range(4):
            # フレーム
            m_frame = ttk.Frame(motors_frame)
            m_frame.pack(fill=tk.X, pady=5)
            
            ttk.Label(m_frame, text=f"M{motor_idx}", width=3, font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            
            # 目標値
            ttk.Label(m_frame, text="目標:").pack(side=tk.LEFT, padx=5)
            var_target = tk.StringVar(value="0")
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
        param_frame = ttk.LabelFrame(self.root, text="パラメータ設定 (100ms ローテーション送信)", padding=10)
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
            
            self.param_vars.append({
                "p": var_p, "i": var_i, "d": var_d, "wheel": var_wheel,
            })

        # ─── TX 有効化 ───────────────────────
        ctrl_frame = ttk.Frame(self.root)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.var_tx_enabled = tk.BooleanVar(value=False)
        self.cb_tx = ttk.Checkbutton(ctrl_frame, text="送信有効化", variable=self.var_tx_enabled,
                                    command=self._on_tx_toggle, state=tk.DISABLED)
        self.cb_tx.pack(side=tk.LEFT)

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
        self.cb_tx.config(state=tk.DISABLED)
        self.var_tx_enabled.set(False)

    def _on_tx_toggle(self):
        if self.backend:
            self.backend.tx_enabled = self.var_tx_enabled.get()
            status = "有効" if self.backend.tx_enabled else "無効"
            self._log(f"📡 送信が {status} になりました")

    def _on_motor_change(self, *args):
        if self.backend:
            for idx, m_var in enumerate(self.motor_vars):
                try:
                    target = int(m_var["target"].get())
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
                    self.backend.set_param(idx, p, i, d, wheel)
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
        
        self.root.after(100, self._update_log)

    def _log(self, msg: str):
        try:
            self.log_queue.put_nowait(msg)
        except queue.Full:
            pass


# ───────────────────────────────────────────────
# メイン
# ───────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = AltairGUI(root)
    root.mainloop()
