#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Altair Module System Unified GUI (Ubuntu対応)
モジュール: MDD, Servo, Solenoid Valve
通信: CAN (socketcan, slcan 等)
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


# =============================================================================
# 定数定義
# =============================================================================

# -- MDD --
CAN_ID_PARAM_BASE = 0x200  # Motor1-4: 0x200-0x203
CAN_ID_MODE       = 0x210  # 4B モード設定
CAN_ID_TARGET     = 0x220  # 8B 目標値設定
CAN_ID_STATUS     = 0x230  # 6B ステータス返信

APP_MODE_PARAMETER = 0
APP_MODE_CONTROL = 1

CONTROL_SPEED = 0
CONTROL_ANGLE = 1
CONTROL_POSITION = 2
MODE_NAMES = {CONTROL_SPEED: "速度", CONTROL_ANGLE: "角度", CONTROL_POSITION: "位置"}

ERR_INIT_TIMEOUT = 0x01
ERR_CAN_RX_TIMEOUT = 0x02
ERR_CAN_TX_FAIL = 0x04

# -- Servo --
CAN_ID_SERVO = 0x100

# -- Solenoid --
CAN_ID_SOLENOID = 0x300

# -- 通信周期 --
TX_INTERVAL_10MS = 0.010
TX_INTERVAL_100MS = 0.100
RX_TIMEOUT_FAST = 0.005

STATUS_LOG_MIN_INTERVAL = 0.2
TARGET_LOG_MIN_INTERVAL = 0.2


# =============================================================================
# 統合 CAN バックエンド
# =============================================================================
class UnifiedCANBackend:
    def __init__(self, interface: str, channel: str, bitrate: int, log_queue: queue.Queue):
        self.interface = interface
        self.channel   = channel
        self.bitrate   = bitrate
        self.log_queue = log_queue
        
        self.bus     = None
        self.running = False
        self.lock    = threading.Lock()

        # --- MDD State ---
        self._mdd_motors = [
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
            {"target": 0, "mode": CONTROL_SPEED, "p": 10, "i": 0, "d": 0, "wheel": 65, "dir": 1},
        ]
        self.mdd_tx_enabled = False
        self.mdd_param_send_requested = False
        self.mdd_remote_app_mode = APP_MODE_PARAMETER
        self.mdd_last_remote_app_mode = None
        
        self._mdd_last_status_time = 0.0
        self._mdd_last_status_log_time = 0.0
        self._mdd_last_status_signature = None
        self._mdd_last_target_log_time = 0.0
        self._mdd_last_target_signature = None

        # --- Servo State ---
        self._servo_targets = [90, 90, 90, 90, 90, 90]
        self.servo_tx_enabled = False

        # --- Solenoid State ---
        self._solenoid_state = 0  # uint16
        self.solenoid_tx_enabled = False


    # ---------------------------------------------------------
    # MDD Setters/Getters
    # ---------------------------------------------------------
    def mdd_set_target(self, motor_index: int, val: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._mdd_motors[motor_index]["target"] = max(-32767, min(32767, int(val)))

    def mdd_set_mode(self, motor_index: int, mode: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._mdd_motors[motor_index]["mode"] = mode

    def mdd_set_param(self, motor_index: int, p: float, i: float, d: float, wheel: float, direction: int):
        with self.lock:
            if 0 <= motor_index < 4:
                self._mdd_motors[motor_index]["p"] = p
                self._mdd_motors[motor_index]["i"] = i
                self._mdd_motors[motor_index]["d"] = d
                self._mdd_motors[motor_index]["wheel"] = wheel
                self._mdd_motors[motor_index]["dir"] = 1 if direction >= 0 else -1

    def mdd_get_motor(self, index: int):
        with self.lock:
            return dict(self._mdd_motors[index])

    def mdd_request_parameter_send(self):
        self.mdd_param_send_requested = True
        self._log("📡 [MDD] パラメータ設定送信を開始します")


    # ---------------------------------------------------------
    # Servo Setters
    # ---------------------------------------------------------
    def servo_set_target(self, index: int, val: int):
        with self.lock:
            if 0 <= index < 6:
                self._servo_targets[index] = max(0, min(180, int(val)))


    # ---------------------------------------------------------
    # Solenoid Setters
    # ---------------------------------------------------------
    def solenoid_set_valve(self, index: int, val: bool):
        with self.lock:
            if 0 <= index < 12:
                if val:
                    self._solenoid_state |= (1 << index)
                else:
                    self._solenoid_state &= ~(1 << index)


    # ---------------------------------------------------------
    # Connection
    # ---------------------------------------------------------
    def connect(self) -> bool:
        if not CAN_AVAILABLE:
            self._log("❌ python-can がインストールされていません → pip install python-can")
            return False
        try:
            self.bus = can.interface.Bus(interface=self.interface, channel=self.channel, bitrate=self.bitrate)
            self._log(f"✅ 接続成功: {self.interface} / {self.channel} @ {self.bitrate} bps")
            return True
        except Exception as e:
            self._log(f"❌ CAN 接続失敗: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self._log("🔌 切断しました")

    def start(self):
        self.running = True
        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def stop(self):
        self.running = False


    # ---------------------------------------------------------
    # TX Loop
    # ---------------------------------------------------------
    def _tx_loop(self):
        next_time_10ms = time.perf_counter()
        next_time_100ms = next_time_10ms

        while self.running:
            now = time.perf_counter()
            
            if now >= next_time_10ms:
                if self.bus:
                    try:
                        self._mdd_send()
                        if self.servo_tx_enabled:
                            self.servo_send_once()
                    except Exception as e:
                        self._log(f"⚠️ 10ms送信エラー: {e}")
                next_time_10ms += TX_INTERVAL_10MS
            
            if now >= next_time_100ms:
                if self.bus:
                    try:
                        if self.solenoid_tx_enabled:
                            self.solenoid_send_once()
                    except Exception as e:
                        self._log(f"⚠️ 100ms送信エラー: {e}")
                next_time_100ms += TX_INTERVAL_100MS

            time.sleep(0.001)

    # -- MDD TX --
    def _mdd_send(self):
        if self.mdd_remote_app_mode == APP_MODE_PARAMETER:
            if self.mdd_param_send_requested:
                for motor_index in range(4):
                    self._mdd_send_param_frame(motor_index)
                self._mdd_send_mode_frame()
        elif self.mdd_remote_app_mode == APP_MODE_CONTROL:
            if self.mdd_tx_enabled:
                self._mdd_send_target_frame()

    @staticmethod
    def _encode_i16_le(value: int):
        value &= 0xFFFF
        return value & 0xFF, (value >> 8) & 0xFF

    def _mdd_send_mode_frame(self):
        payload = bytearray(4)
        with self.lock:
            for i in range(4):
                payload[i] = self._mdd_motors[i]["mode"] & 0xFF
        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_MODE, data=payload)
        self.bus.send(msg)

    def _mdd_send_target_frame(self):
        payload = bytearray(8)
        with self.lock:
            for i in range(4):
                scaled_target = int(round(float(self._mdd_motors[i]["target"]) * 10.0))
                scaled_target = max(-32768, min(32767, scaled_target))
                low, high = self._encode_i16_le(scaled_target)
                payload[i * 2] = low
                payload[i * 2 + 1] = high
            target_signature = tuple(int(self._mdd_motors[i]["target"]) for i in range(4))

        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_TARGET, data=payload[:8])
        self.bus.send(msg)

        now = time.perf_counter()
        if (target_signature != self._mdd_last_target_signature) or ((now - self._mdd_last_target_log_time) >= TARGET_LOG_MIN_INTERVAL):
            self._log(f"📤 [MDD] 0x220(target) 送信: target={list(target_signature)}")
            self._mdd_last_target_signature = target_signature
            self._mdd_last_target_log_time = now

    def _mdd_send_param_frame(self, motor_index: int):
        payload = bytearray(8)
        with self.lock:
            motor = self._mdd_motors[motor_index]
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

        payload[0] = p_low; payload[1] = p_high
        payload[2] = i_low; payload[3] = i_high
        payload[4] = d_low; payload[5] = d_high
        payload[6] = w_low; payload[7] = w_high

        msg = can.Message(is_extended_id=False, arbitration_id=CAN_ID_PARAM_BASE + motor_index, data=payload)
        self.bus.send(msg)

    # -- Servo TX --
    def servo_send_once(self):
        if self.bus:
            with self.lock:
                data = bytes(self._servo_targets)
            msg = can.Message(arbitration_id=CAN_ID_SERVO, data=data, is_extended_id=False)
            self.bus.send(msg)

    # -- Solenoid TX --
    def solenoid_send_once(self):
        if self.bus:
            with self.lock:
                data0 = self._solenoid_state & 0xFF
                data1 = (self._solenoid_state >> 8) & 0xFF
                data = bytes([data0, data1])
            msg = can.Message(arbitration_id=CAN_ID_SOLENOID, data=data, is_extended_id=False)
            self.bus.send(msg)


    # ---------------------------------------------------------
    # RX Loop (mainly for MDD status)
    # ---------------------------------------------------------
    def _rx_loop(self):
        while self.running:
            if self.bus:
                try:
                    msg = self.bus.recv(timeout=RX_TIMEOUT_FAST)
                    while msg is not None:
                        if msg.arbitration_id == CAN_ID_STATUS:
                            self._mdd_parse_status(msg.data)
                        msg = self.bus.recv(timeout=0.0)
                except Exception:
                    pass

    def _mdd_parse_status(self, data):
        if len(data) < 6:
            return

        limit_states = [1 if data[i] else 0 for i in range(4)]
        error_code = data[4]
        app_mode = data[5] & 0x01
        previous_mode = self.mdd_remote_app_mode if self.mdd_last_remote_app_mode is not None else None

        self.mdd_last_remote_app_mode = self.mdd_remote_app_mode
        self.mdd_remote_app_mode = app_mode
        self._mdd_last_status_time = time.perf_counter()

        if previous_mode == APP_MODE_PARAMETER and app_mode == APP_MODE_CONTROL:
            self.mdd_param_send_requested = False
            self._log("✅ [MDD] 制御実行モードへ移行しました。パラメータ送信を停止します")

        if previous_mode == APP_MODE_CONTROL and app_mode == APP_MODE_PARAMETER:
            self.mdd_param_send_requested = False
            self.mdd_tx_enabled = False
            self._log("⛔ [MDD] 制御実行からパラメータ設定へ戻りました。送信を停止しました")

        now = time.perf_counter()
        signature = (tuple(limit_states), error_code, app_mode)
        if (signature != self._mdd_last_status_signature) or ((now - self._mdd_last_status_log_time) >= STATUS_LOG_MIN_INTERVAL):
            limit_str = ", ".join([f"SW{i+1}={state}" for i, state in enumerate(limit_states)])
            sys_str = "制御実行" if app_mode == APP_MODE_CONTROL else "パラメータ設定"
            self._log(f"📥 [MDD] 0x230: {limit_str} | State={sys_str} | Err=0x{error_code:02X}")
            self._mdd_last_status_signature = signature
            self._mdd_last_status_log_time = now

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{ts}] {msg}")


# =============================================================================
# GUI アプリケーション
# =============================================================================
class AltairUnifiedGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Altair Module System Unified GUI (Ubuntu対応)")
        self.root.geometry("1100x800")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        
        self.log_queue = queue.Queue()
        self.backend = None
        
        self._build_ui()
        self._poll_log()

    def _build_ui(self):
        # -- 接続パネル --
        conn_frame = ttk.LabelFrame(self.root, text="CAN 接続 (Ubuntu用)", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Interface:").grid(row=0, column=0, sticky=tk.W)
        self.var_interface = tk.StringVar(value="socketcan")
        ttk.Combobox(conn_frame, textvariable=self.var_interface, 
                     values=("socketcan", "slcan", "pcan", "ixxat", "vector", "virtual"), width=12).grid(row=0, column=1, padx=5)
        
        ttk.Label(conn_frame, text="Channel:").grid(row=0, column=2, sticky=tk.W, padx=(10, 0))
        self.var_channel = tk.StringVar(value="can0")
        ttk.Entry(conn_frame, textvariable=self.var_channel, width=15).grid(row=0, column=3, sticky=tk.W, padx=5)
        
        ttk.Label(conn_frame, text="Bitrate:").grid(row=0, column=4, sticky=tk.W, padx=(10, 0))
        self.var_bitrate = tk.StringVar(value="1000000")
        ttk.Combobox(conn_frame, textvariable=self.var_bitrate, 
                     values=["125000", "250000", "500000", "1000000"], width=10).grid(row=0, column=5, padx=5)
        
        self.btn_connect = ttk.Button(conn_frame, text="接続", command=self._on_connect)
        self.btn_connect.grid(row=0, column=6, padx=15)
        
        self.btn_disconnect = ttk.Button(conn_frame, text="切断", command=self._on_disconnect, state=tk.DISABLED)
        self.btn_disconnect.grid(row=0, column=7)

        # -- NOTEBOOK (タブ) --
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self._build_tab_mdd()
        self._build_tab_servo()
        self._build_tab_solenoid()

        # -- ログビュー --
        log_frame = ttk.LabelFrame(self.root, text="統合ログ", padding=5)
        log_frame.pack(fill=tk.X, padx=10, pady=5)

        # ログクリアボタン
        log_ctrl_frame = ttk.Frame(log_frame)
        log_ctrl_frame.pack(fill=tk.X)
        ttk.Button(log_ctrl_frame, text="ログクリア", command=self._clear_log).pack(side=tk.RIGHT)

        self.txt_log = scrolledtext.ScrolledText(log_frame, height=10, state=tk.DISABLED, bg="#F9F9F9")
        self.txt_log.pack(fill=tk.BOTH, expand=True)

    # ---------------------------------------------------------
    # TAB: MDD
    # ---------------------------------------------------------
    def _build_tab_mdd(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text=" MDD (0x200-0x230) ")

        # モータ制御パネル
        motors_frame = ttk.LabelFrame(tab, text="モータ制御 (制御実行モード時のみ 10ms送信)", padding=10)
        motors_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.mdd_motor_vars = []
        for i in range(4):
            f = ttk.Frame(motors_frame)
            f.pack(fill=tk.X, pady=5)
            
            ttk.Label(f, text=f"M{i}", width=3, font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            
            ttk.Label(f, text="目標:").pack(side=tk.LEFT, padx=5)
            var_t = tk.DoubleVar(value=0.0)
            s = ttk.Scale(f, from_=-32767, to=32767, variable=var_t, orient=tk.HORIZONTAL, command=self._on_mdd_change)
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            e = ttk.Entry(f, textvariable=var_t, width=8)
            e.pack(side=tk.LEFT, padx=5)
            
            ttk.Label(f, text="モード:").pack(side=tk.LEFT, padx=5)
            var_m = tk.IntVar(value=CONTROL_SPEED)
            for v, n in MODE_NAMES.items():
                ttk.Radiobutton(f, text=n, variable=var_m, value=v, command=self._on_mdd_change).pack(side=tk.LEFT)
            
            self.mdd_motor_vars.append({"target": var_t, "mode": var_m, "slider": s, "entry": e})

        # パラメータパネル
        param_frame = ttk.LabelFrame(tab, text="パラメータ設定 (設定送信ボタン押下後 10ms送信)", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.mdd_param_vars = []
        for i in range(4):
            f = ttk.Frame(param_frame)
            f.pack(fill=tk.X, pady=3)
            
            ttk.Label(f, text=f"M{i}", width=3).pack(side=tk.LEFT)
            ttk.Label(f, text="P:").pack(side=tk.LEFT, padx=2)
            vp = tk.StringVar(value="80"); ttk.Entry(f, textvariable=vp, width=6).pack(side=tk.LEFT)
            ttk.Label(f, text="I:").pack(side=tk.LEFT, padx=2)
            vi = tk.StringVar(value="0"); ttk.Entry(f, textvariable=vi, width=6).pack(side=tk.LEFT)
            ttk.Label(f, text="D:").pack(side=tk.LEFT, padx=2)
            vd = tk.StringVar(value="2"); ttk.Entry(f, textvariable=vd, width=6).pack(side=tk.LEFT)
            ttk.Label(f, text="Wheel(mm):").pack(side=tk.LEFT, padx=2)
            vw = tk.StringVar(value="65"); ttk.Entry(f, textvariable=vw, width=6).pack(side=tk.LEFT)
            ttk.Label(f, text="Dir:").pack(side=tk.LEFT, padx=2)
            vdir = tk.IntVar(value=1)
            ttk.Radiobutton(f, text="+", variable=vdir, value=1, command=self._on_mdd_change).pack(side=tk.LEFT)
            ttk.Radiobutton(f, text="-", variable=vdir, value=-1, command=self._on_mdd_change).pack(side=tk.LEFT)
            
            self.mdd_param_vars.append({"p": vp, "i": vi, "d": vd, "wheel": vw, "dir": vdir})

        # 制御ボトム
        ctrl_frame = ttk.Frame(tab)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(ctrl_frame, text="MDD状態:").pack(side=tk.LEFT)
        self.var_mdd_state = tk.StringVar(value="未接続")
        ttk.Label(ctrl_frame, textvariable=self.var_mdd_state, font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(4, 16))

        self.btn_mdd_param_send = ttk.Button(ctrl_frame, text="設定送信", command=self._on_mdd_start_param_send, state=tk.DISABLED)
        self.btn_mdd_param_send.pack(side=tk.LEFT)

        self.var_mdd_tx = tk.BooleanVar(value=False)
        self.cb_mdd_tx = ttk.Checkbutton(ctrl_frame, text="制御指令送信有効化", variable=self.var_mdd_tx,
                                         command=self._on_mdd_tx_toggle, state=tk.DISABLED)
        self.cb_mdd_tx.pack(side=tk.LEFT, padx=(16, 0))


    # ---------------------------------------------------------
    # TAB: Servo
    # ---------------------------------------------------------
    def _build_tab_servo(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text=" Servo (0x100) ")

        f1 = ttk.Frame(tab)
        f1.pack(pady=10)

        self.var_servo_tx = tk.BooleanVar(value=False)
        self.cb_servo_tx = ttk.Checkbutton(f1, text="送信 ON (10ms周期)", variable=self.var_servo_tx, 
                                           command=self._on_servo_tx_toggle, state=tk.DISABLED)
        self.cb_servo_tx.pack(side=tk.LEFT, padx=10)
        
        self.btn_servo_once = ttk.Button(f1, text="今すぐ1回送信", command=self._on_servo_send_once, state=tk.DISABLED)
        self.btn_servo_once.pack(side=tk.LEFT, padx=10)

        f2 = ttk.Frame(tab)
        f2.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        self.servo_vars = []
        for i in range(6):
            col = i * 2
            ttk.Label(f2, text=f"CH{i+1}\n(0-180)", justify=tk.CENTER).grid(row=0, column=col, padx=10)
            
            var = tk.IntVar(value=90)
            self.servo_vars.append(var)
            
            s = ttk.Scale(f2, from_=180, to=0, orient=tk.VERTICAL, variable=var, length=200, command=lambda val, idx=i: self._on_servo_change(idx))
            s.grid(row=1, column=col, pady=10)
            
            sb = ttk.Spinbox(f2, from_=0, to=180, textvariable=var, width=5, command=lambda idx=i: self._on_servo_change(idx))
            sb.grid(row=2, column=col)
            sb.bind("<Return>", lambda e, idx=i: self._on_servo_change(idx))

    # ---------------------------------------------------------
    # TAB: Solenoid Valve
    # ---------------------------------------------------------
    def _build_tab_solenoid(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text=" Solenoid Valve (0x300) ")

        f1 = ttk.Frame(tab)
        f1.pack(pady=10)

        self.var_sol_tx = tk.BooleanVar(value=False)
        self.cb_sol_tx = ttk.Checkbutton(f1, text="送信 ON (100ms周期)", variable=self.var_sol_tx,
                                         command=self._on_solenoid_tx_toggle, state=tk.DISABLED)
        self.cb_sol_tx.pack(side=tk.LEFT, padx=10)

        self.btn_sol_once = ttk.Button(f1, text="今すぐ1回送信", command=self._on_solenoid_send_once, state=tk.DISABLED)
        self.btn_sol_once.pack(side=tk.LEFT, padx=10)

        f2 = ttk.Frame(tab)
        f2.pack(pady=20)
        
        valve_pins = ["A0", "A1", "A6", "A7", "A8", "A9", "A15", "B3", "B6", "B7", "B8", "B9"]
        self.solenoid_vars = []
        for i in range(12):
            col = i % 6
            row = i // 6
            var = tk.BooleanVar(value=False)
            self.solenoid_vars.append(var)

            cell = ttk.Frame(f2)
            cell.grid(row=row, column=col, padx=15, pady=10)
            
            ttk.Label(cell, text=f"Valve {i+1}\n({valve_pins[i]})", justify=tk.CENTER).pack()
            cb = ttk.Checkbutton(cell, variable=var, command=lambda idx=i: self._on_solenoid_change(idx))
            cb.pack(pady=4)

    # ---------------------------------------------------------
    # Logic Actions
    # ---------------------------------------------------------
    def _on_connect(self):
        interface = self.var_interface.get().strip()
        channel = self.var_channel.get().strip()
        try:
            bitrate = int(self.var_bitrate.get())
        except:
            messagebox.showerror("エラー", "Bitrateは整数で指定してください")
            return

        self.backend = UnifiedCANBackend(interface, channel, bitrate, self.log_queue)
        if self.backend.connect():
            self.backend.start()
            
            self.btn_connect.config(state=tk.DISABLED)
            self.btn_disconnect.config(state=tk.NORMAL)
            
            # Enable MDD UI
            self.btn_mdd_param_send.config(state=tk.NORMAL)
            self.cb_mdd_tx.config(state=tk.NORMAL)
            
            # Enable Servo UI
            self.cb_servo_tx.config(state=tk.NORMAL)
            self.btn_servo_once.config(state=tk.NORMAL)
            
            # Enable Solenoid UI
            self.cb_sol_tx.config(state=tk.NORMAL)
            self.btn_sol_once.config(state=tk.NORMAL)
            
            # sync current states
            self._on_mdd_change()
            for i in range(6): self._on_servo_change(i)
            for i in range(12): self._on_solenoid_change(i)
        else:
            self.backend = None

    def _on_disconnect(self):
        if self.backend:
            self.backend.disconnect()
            self.backend = None
        
        self.btn_connect.config(state=tk.NORMAL)
        self.btn_disconnect.config(state=tk.DISABLED)
        
        self.btn_mdd_param_send.config(state=tk.DISABLED)
        self.cb_mdd_tx.config(state=tk.DISABLED)
        self.var_mdd_tx.set(False)

        self.cb_servo_tx.config(state=tk.DISABLED)
        self.btn_servo_once.config(state=tk.DISABLED)
        self.var_servo_tx.set(False)

        self.cb_sol_tx.config(state=tk.DISABLED)
        self.btn_sol_once.config(state=tk.DISABLED)
        self.var_sol_tx.set(False)

        self.var_mdd_state.set("未接続")


    # -- MDD Actions --
    def _on_mdd_start_param_send(self):
        if self.backend:
            self._on_mdd_change()
            self.backend.mdd_request_parameter_send()

    def _on_mdd_tx_toggle(self):
        if self.backend:
            self.backend.mdd_tx_enabled = self.var_mdd_tx.get()

    def _on_mdd_change(self, *args):
        if self.backend:
            for idx, m in enumerate(self.mdd_motor_vars):
                try: t = float(m["target"].get())
                except: t = 0
                self.backend.mdd_set_target(idx, t)
                self.backend.mdd_set_mode(idx, m["mode"].get())
            for idx, p in enumerate(self.mdd_param_vars):
                try:
                    self.backend.mdd_set_param(
                        idx, float(p["p"].get()), float(p["i"].get()), float(p["d"].get()),
                        float(p["wheel"].get()), int(p["dir"].get())
                    )
                except: pass

    # -- Servo Actions --
    def _on_servo_tx_toggle(self):
        if self.backend:
            self.backend.servo_tx_enabled = self.var_servo_tx.get()

    def _on_servo_send_once(self):
        if self.backend:
            self.backend.servo_send_once()

    def _on_servo_change(self, idx):
        if self.backend:
            try:
                v = self.servo_vars[idx].get()
                self.backend.servo_set_target(idx, v)
            except: pass

    # -- Solenoid Actions --
    def _on_solenoid_tx_toggle(self):
        if self.backend:
            self.backend.solenoid_tx_enabled = self.var_sol_tx.get()

    def _on_solenoid_send_once(self):
        if self.backend:
            self.backend.solenoid_send_once()

    def _on_solenoid_change(self, idx):
        val = self.solenoid_vars[idx].get()
        if self.backend:
            self.backend.solenoid_set_valve(idx, val)

    # -- Log --
    def _clear_log(self):
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.delete("1.0", tk.END)
        self.txt_log.config(state=tk.DISABLED)

    def _poll_log(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.txt_log.config(state=tk.NORMAL)
            self.txt_log.insert(tk.END, msg + "\n")
            self.txt_log.see(tk.END)
            self.txt_log.config(state=tk.DISABLED)

        if self.backend:
            # Sync MDD status text
            if self.backend.mdd_remote_app_mode == APP_MODE_CONTROL:
                self.var_mdd_state.set("🟢 制御実行")
            else:
                self.var_mdd_state.set("🟡 パラメータ設定")
            
            if self.var_mdd_tx.get() != self.backend.mdd_tx_enabled:
                self.var_mdd_tx.set(self.backend.mdd_tx_enabled)
        else:
            self.var_mdd_state.set("未接続")
            
        self.root.after(100, self._poll_log)

    def _on_close(self):
        if self.backend:
            self.backend.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = AltairUnifiedGUI(root)
    root.mainloop()
