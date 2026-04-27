#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
電磁弁(solenoid_valve) CAN送信 GUIツール (Ubuntu対応)

CAN ID : 0x300
Payload: 2byte (12個の電磁弁のON/OFF状態をビットアサイン)
         byte[0]: Valve 1-8
         byte[1]: Valve 9-12
送信周期: 100ms
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import queue
from datetime import datetime

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

# ─────────────────────────────────────────────
# 定数
# ─────────────────────────────────────────────
CAN_ID_SOLENOID = 0x300
TX_INTERVAL     = 0.100   # 100ms

# ─────────────────────────────────────────────
# CAN通信バックエンド
# ─────────────────────────────────────────────
class CANBackend:
    def __init__(self, interface: str, channel: str, bitrate: int, log_queue: queue.Queue):
        self.interface  = interface
        self.channel    = channel
        self.bitrate    = bitrate
        self.log_queue  = log_queue
        self.bus        = None
        self.running    = False
        self.lock       = threading.Lock()

        self._state     = 0  # uint16 (12 valves)
        self.tx_enabled = False

    def set_valve(self, index: int, val: bool):
        with self.lock:
            if val:
                self._state |= (1 << index)
            else:
                self._state &= ~(1 << index)

    def connect(self) -> bool:
        if not CAN_AVAILABLE:
            self._log("❌ python-can がインストールされていません  →  pip install python-can")
            return False
        try:
            self.bus = can.interface.Bus(
                interface=self.interface,
                channel=self.channel,
                bitrate=self.bitrate,
            )
            self._log(f"✅ 接続成功: {self.interface} / {self.channel} @ {self.bitrate} bps")
            return True
        except Exception as e:
            self._log(f"❌ 接続失敗: {e}")
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

    def stop(self):
        self.running = False

    def _tx_loop(self):
        next_time = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            if now >= next_time:
                if self.bus and self.tx_enabled:
                    try:
                        self._send_command()
                    except Exception as e:
                        self._log(f"⚠️ 送信エラー: {e}")
                next_time += TX_INTERVAL
            else:
                time.sleep(0.001)

    def send_once(self):
        """単発送信"""
        self._send_command()

    def _send_command(self):
        with self.lock:
            data0 = self._state & 0xFF
            data1 = (self._state >> 8) & 0xFF
            data = bytes([data0, data1])
        msg = can.Message(arbitration_id=CAN_ID_SOLENOID, data=data, is_extended_id=False)
        if self.bus:
            self.bus.send(msg)

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{ts}] {msg}")


# ─────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────
class SolenoidCanGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("0x300 電磁弁 CAN送信ツール (Ubuntu対応)")
        self.resizable(True, True)

        self.log_queue = queue.Queue()
        self.backend   = None
        self.connected = False

        self._build_ui()
        self._poll_log()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        self._build_connection_frame(main)
        self._build_solenoid_frame(main)
        self._build_log_frame(main)

    def _build_connection_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="CAN接続設定 (Ubuntu用)", padding=6)
        frm.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(frm, text="Interface:").grid(row=0, column=0, sticky=tk.W, padx=4)
        self.var_iface = tk.StringVar(value="socketcan")
        ttk.Combobox(frm, textvariable=self.var_iface, width=14,
                     values=["socketcan", "slcan", "pcan", "ixxat", "vector", "virtual"]
                     ).grid(row=0, column=1, padx=4)

        ttk.Label(frm, text="Channel:").grid(row=0, column=2, sticky=tk.W, padx=4)
        self.var_channel = tk.StringVar(value="can0")
        ttk.Entry(frm, textvariable=self.var_channel, width=10).grid(row=0, column=3, padx=4)

        ttk.Label(frm, text="Bitrate:").grid(row=0, column=4, sticky=tk.W, padx=4)
        self.var_bitrate = tk.StringVar(value="1000000")
        ttk.Combobox(frm, textvariable=self.var_bitrate, width=10,
                     values=["125000", "250000", "500000", "1000000"]
                     ).grid(row=0, column=5, padx=4)

        self.btn_connect = ttk.Button(frm, text="接続", command=self._toggle_connect, width=10)
        self.btn_connect.grid(row=0, column=6, padx=(12, 4))

        self.lbl_conn_state = ttk.Label(frm, text="● 未接続", foreground="gray")
        self.lbl_conn_state.grid(row=0, column=7, padx=4)

    def _build_solenoid_frame(self, parent):
        outer = ttk.LabelFrame(
            parent,
            text=f"電磁弁指令  |  CAN ID: 0x{CAN_ID_SOLENOID:03X}  |  12ch",
            padding=10)
        outer.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        # 送信ON/OFF + 単発送信
        top = ttk.Frame(outer)
        top.pack(fill=tk.X, pady=(0, 6))
        self.var_tx = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="送信 ON  (100ms周期)",
                        variable=self.var_tx,
                        command=self._on_tx_toggle).pack(side=tk.LEFT)
        ttk.Button(top, text="今すぐ1回送信",
                   command=self._send_once).pack(side=tk.LEFT, padx=12)

        ttk.Separator(outer, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(4, 10))

        # 12ch チェックボックス
        self.valve_vars = []
        ch_frm = ttk.Frame(outer)
        ch_frm.pack(fill=tk.X)

        valve_pins = ["A0", "A1", "A6", "A7", "A8", "A9", "A15", "B3", "B6", "B7", "B8", "B9"]

        for i in range(12):
            col = i % 6
            row = i // 6
            var = tk.BooleanVar(value=False)
            self.valve_vars.append(var)

            frm = ttk.Frame(ch_frm)
            frm.grid(row=row, column=col, padx=10, pady=5)
            
            ttk.Label(frm, text=f"Valve {i+1}\n({valve_pins[i]})", justify=tk.CENTER).pack()
            cb = ttk.Checkbutton(frm, variable=var, command=lambda idx=i: self._on_valve_change(idx))
            cb.pack(pady=4)

    def _build_log_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="ログ表示", padding=6)
        frm.pack(fill=tk.BOTH, expand=True)

        self.txt_log = scrolledtext.ScrolledText(frm, height=8, width=70, state=tk.DISABLED, bg="#F9F9F9")
        self.txt_log.pack(fill=tk.BOTH, expand=True)

    def _toggle_connect(self):
        if not self.connected:
            iface   = self.var_iface.get().strip()
            channel = self.var_channel.get().strip()
            try:
                bitrate = int(self.var_bitrate.get().strip())
            except:
                self._log("❌ Bitrate が不正です")
                return

            self.backend = CANBackend(iface, channel, bitrate, self.log_queue)
            ok = self.backend.connect()
            if ok:
                self.backend.start()
                self.connected = True
                self.btn_connect.config(text="切断")
                self.lbl_conn_state.config(text="● 接続済", foreground="green")

                # 現在の送信ON/OFFチェックボックス・バルブ状態をバックエンドに反映
                self.backend.tx_enabled = self.var_tx.get()
                for i, var in enumerate(self.valve_vars):
                    self.backend.set_valve(i, var.get())
            else:
                self.backend = None
        else:
            if self.backend:
                self.backend.disconnect()
            self.backend = None
            self.connected = False
            self.btn_connect.config(text="接続")
            self.lbl_conn_state.config(text="● 未接続", foreground="gray")

    def _on_tx_toggle(self):
        if self.backend:
            self.backend.tx_enabled = self.var_tx.get()
            self._log(f"🔄 送信状態変更: {'ON' if self.var_tx.get() else 'OFF'}")

    def _send_once(self):
        if self.backend and self.connected:
            self.backend.send_once()
            self._log("▶ 単発送信")
        else:
            self._log("⚠️ 未接続なので送信できません")

    def _on_valve_change(self, idx):
        val = self.valve_vars[idx].get()
        if self.backend:
            self.backend.set_valve(idx, val)

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{ts}] {msg}")

    def _poll_log(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.txt_log.config(state=tk.NORMAL)
            self.txt_log.insert(tk.END, msg + "\n")
            self.txt_log.see(tk.END)
            self.txt_log.config(state=tk.DISABLED)
        self.after(100, self._poll_log)

    def _on_close(self):
        if self.backend:
            self.backend.disconnect()
        self.destroy()

if __name__ == "__main__":
    app = SolenoidCanGUI()
    app.mainloop()
