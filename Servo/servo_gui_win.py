#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
0x100 サーボ指令 CAN送信 GUIツール (Windows対応)
USBtoCAN経由で 6ch サーボ指令を 10ms周期で送信する

CAN ID : 0x100
Payload: 6byte (uint8 × 6, 各0~180)
送信周期: 10ms
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

# ─────────────────────────────────────────────
# 定数
# ─────────────────────────────────────────────
CAN_ID_SERVO     = 0x100
TX_INTERVAL_10MS = 0.010   # 10ms


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

        self._servo     = [0, 0, 0, 0, 0, 0]  # 各0~180
        self.tx_enabled = False

    # ── セッター ────────────────────────────────
    def set_servo(self, index: int, val: int):
        with self.lock:
            self._servo[index] = max(0, min(180, int(val)))

    # ── 接続 ────────────────────────────────────
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

    # ── スレッド起動 ─────────────────────────────
    def start(self):
        self.running = True
        threading.Thread(target=self._tx_loop, daemon=True).start()

    def stop(self):
        self.running = False

    # ── 10ms 送信ループ ──────────────────────────
    def _tx_loop(self):
        next_time = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            if now >= next_time:
                if self.bus and self.tx_enabled:
                    try:
                        self._send_servo()
                    except Exception as e:
                        self._log(f"⚠️ 送信エラー: {e}")
                next_time += TX_INTERVAL_10MS
            else:
                time.sleep(0.0001)

    # ── 送信 ────────────────────────────────────
    def send_once(self):
        """単発送信"""
        self._send_servo()

    def _send_servo(self):
        with self.lock:
            data = bytes(self._servo)
        msg = can.Message(arbitration_id=CAN_ID_SERVO, data=data, is_extended_id=False)
        self.bus.send(msg)

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{ts}] {msg}")


# ─────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────
class ServoCanGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("0x100 サーボ指令 CAN送信ツール (Windows対応)")
        self.resizable(True, True)

        self.log_queue = queue.Queue()
        self.backend   = None
        self.connected = False

        self._build_ui()
        self._poll_log()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ═══════════════════════════════════════════
    # UI構築
    # ═══════════════════════════════════════════
    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        self._build_connection_frame(main)
        self._build_servo_frame(main)
        self._build_log_frame(main)

    # ── 接続設定 ──────────────────────────────────
    def _build_connection_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="CAN接続設定 (Windows用)", padding=6)
        frm.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(frm, text="Interface:").grid(row=0, column=0, sticky=tk.W, padx=4)
        self.var_iface = tk.StringVar(value="slcan")
        ttk.Combobox(frm, textvariable=self.var_iface, width=14,
                     values=["slcan", "pcan", "ixxat", "vector", "kvaser", "virtual"]
                     ).grid(row=0, column=1, padx=4)

        ttk.Label(frm, text="Channel:").grid(row=0, column=2, sticky=tk.W, padx=4)
        self.var_channel = tk.StringVar(value="COM3")
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

    # ── サーボ指令フレーム ────────────────────────
    def _build_servo_frame(self, parent):
        outer = ttk.LabelFrame(
            parent,
            text=f"サーボ指令  |  CAN ID: 0x{CAN_ID_SERVO:03X}  |  6ch  |  各値 0 ~ 180",
            padding=10)
        outer.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        # 送信ON/OFF + 単発送信
        top = ttk.Frame(outer)
        top.pack(fill=tk.X, pady=(0, 6))
        self.var_tx = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="送信 ON  (10ms周期)",
                        variable=self.var_tx,
                        command=self._on_tx_toggle).pack(side=tk.LEFT)
        ttk.Button(top, text="今すぐ1回送信",
                   command=self._send_once).pack(side=tk.LEFT, padx=12)

        ttk.Separator(outer, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(4, 10))

        # 6ch スライダー + スピンボックス
        self.servo_vars = []
        ch_frm = ttk.Frame(outer)
        ch_frm.pack()

        for i in range(6):
            col = i * 2
            var = tk.IntVar(value=0)
            self.servo_vars.append(var)

            ttk.Label(ch_frm, text=f"CH {i+1}",
                      font=("", 11, "bold"), anchor=tk.CENTER,
                      width=6).grid(row=0, column=col, columnspan=2, pady=(0, 4))

            tk.Scale(
                ch_frm, variable=var,
                orient=tk.VERTICAL,
                from_=180, to=0, resolution=1,
                length=200, width=22,
                tickinterval=45,
                command=lambda v, idx=i: self._on_change(idx),
            ).grid(row=1, column=col, columnspan=2, padx=10, pady=2)

            spn = ttk.Spinbox(ch_frm, textvariable=var,
                              from_=0, to=180, increment=1, width=6,
                              command=lambda idx=i: self._on_change(idx))
            spn.grid(row=2, column=col, columnspan=2, padx=10, pady=(2, 6))
            spn.bind("<Return>", lambda e, idx=i: self._on_change(idx))

        # 一括操作
        ttk.Separator(outer, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(4, 8))
        bulk = ttk.LabelFrame(outer, text="一括操作", padding=6)
        bulk.pack(fill=tk.X, padx=4)
        for val, label in [(0, "全CH → 0"), (90, "全CH → 90"), (180, "全CH → 180")]:
            ttk.Button(bulk, text=label,
                       command=lambda v=val: self._set_all(v)).pack(side=tk.LEFT, padx=6)

        # ペイロードプレビュー
        ttk.Separator(outer, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(8, 4))
        self.lbl_preview = ttk.Label(outer, text="", font=("Courier", 10),
                                     justify=tk.LEFT, foreground="#004488")
        self.lbl_preview.pack(anchor=tk.W, padx=4, pady=(0, 4))
        self._update_preview()

    # ── ログ ─────────────────────────────────────
    def _build_log_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="ログ", padding=4)
        frm.pack(fill=tk.BOTH, expand=True)
        self.log_text = scrolledtext.ScrolledText(frm, height=7, state=tk.DISABLED,
                                                   font=("Courier", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        ttk.Button(frm, text="クリア", command=self._clear_log).pack(anchor=tk.E, pady=(2, 0))

    # ═══════════════════════════════════════════
    # イベントハンドラ
    # ═══════════════════════════════════════════
    def _toggle_connect(self):
        if not self.connected:
            self._do_connect()
        else:
            self._do_disconnect()

    def _do_connect(self):
        iface   = self.var_iface.get().strip()
        channel = self.var_channel.get().strip()
        try:
            bitrate = int(self.var_bitrate.get())
        except ValueError:
            messagebox.showerror("入力エラー", "ビットレートが不正です")
            return

        self.backend = CANBackend(iface, channel, bitrate, self.log_queue)
        if self.backend.connect():
            self._sync_to_backend()
            self.backend.start()
            self.connected = True
            self.btn_connect.configure(text="切断")
            self.lbl_conn_state.configure(text="● 接続中", foreground="green")
        else:
            self.backend = None

    def _do_disconnect(self):
        if self.backend:
            self.backend.stop()
            self.backend.disconnect()
            self.backend = None
        self.connected = False
        self.var_tx.set(False)
        self.btn_connect.configure(text="接続")
        self.lbl_conn_state.configure(text="● 未接続", foreground="gray")

    def _on_tx_toggle(self):
        if self.backend:
            self._sync_to_backend()
            self.backend.tx_enabled = self.var_tx.get()
            state = "開始" if self.var_tx.get() else "停止"
            self._log_gui(f"📡 10ms周期送信 {state}")
        else:
            if self.var_tx.get():
                self.var_tx.set(False)
                messagebox.showwarning("未接続", "先にCANバスに接続してください")

    def _on_change(self, idx: int):
        if self.backend:
            self.backend.set_servo(idx, self.servo_vars[idx].get())
        self._update_preview()

    def _send_once(self):
        if not self.connected or not self.backend:
            messagebox.showwarning("未接続", "先にCANバスに接続してください")
            return
        self._sync_to_backend()
        try:
            self.backend.send_once()
            vals = [v.get() for v in self.servo_vars]
            self._log_gui("📤 単発送信: " + "  ".join(f"CH{i+1}={v}" for i, v in enumerate(vals)))
        except Exception as e:
            self._log_gui(f"⚠️ 送信失敗: {e}")

    def _set_all(self, val: int):
        for i, var in enumerate(self.servo_vars):
            var.set(val)
            if self.backend:
                self.backend.set_servo(i, val)
        self._update_preview()

    def _sync_to_backend(self):
        if not self.backend:
            return
        for i, var in enumerate(self.servo_vars):
            self.backend.set_servo(i, var.get())

    def _update_preview(self):
        vals    = [v.get() for v in self.servo_vars]
        hex_str = "  ".join(f"{v:02X}" for v in vals)
        dec_str = "  ".join(f"{v:3d}" for v in vals)
        ch_hdr  = "  ".join(f"CH{i+1} " for i in range(6))
        self.lbl_preview.configure(
            text=f"CAN ID : 0x{CAN_ID_SERVO:03X}\n"
                 f"CH     :  {ch_hdr}\n"
                 f"Payload:  {dec_str}  (DEC)\n"
                 f"          {hex_str}  (HEX)")

    # ═══════════════════════════════════════════
    # ユーティリティ
    # ═══════════════════════════════════════════
    def _log_gui(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{ts}] {msg}")

    def _poll_log(self):
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.log_text.configure(state=tk.NORMAL)
                self.log_text.insert(tk.END, msg + "\n")
                self.log_text.see(tk.END)
                self.log_text.configure(state=tk.DISABLED)
        except queue.Empty:
            pass
        self.after(100, self._poll_log)

    def _clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _on_close(self):
        if self.backend:
            self.backend.stop()
            self.backend.disconnect()
        self.destroy()


# ─────────────────────────────────────────────
# エントリポイント
# ─────────────────────────────────────────────
if __name__ == "__main__":
    if not CAN_AVAILABLE:
        print("⚠️  python-can が見つかりません。")
        print("    pip install python-can  でインストールしてください。")

    app = ServoCanGUI()
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
