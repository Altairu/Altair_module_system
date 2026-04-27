#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Altair Module System Advanced GUI (Ubuntu対応)
動的モジュール追加、可変CAN ID、マクロ・トリガー制御に対応した拡張版
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, simpledialog
import threading
import time
import queue
import json
import os
from datetime import datetime

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False


# =============================================================================
# 定数定義
# =============================================================================
APP_MODE_PARAMETER = 0
APP_MODE_CONTROL = 1

CONTROL_SPEED = 0
CONTROL_ANGLE = 1
CONTROL_POSITION = 2
MODE_NAMES = {CONTROL_SPEED: "速度", CONTROL_ANGLE: "角度", CONTROL_POSITION: "位置"}

CONFIG_FILE = "altair_advanced_config.json"

# =============================================================================
# 統合 CAN バックエンド
# =============================================================================
class AdvancedCANBackend:
    def __init__(self, interface: str, channel: str, bitrate: int, log_queue: queue.Queue, modules: list, automation_rules: list):
        self.interface = interface
        self.channel   = channel
        self.bitrate   = bitrate
        self.log_queue = log_queue
        self.modules = modules # 参照渡し
        self.automation_rules = automation_rules # 参照渡し
        
        self.bus     = None
        self.running = False
        self.lock    = threading.Lock()
        
        self.auto_trigger_engine_enabled = False

    def connect(self) -> bool:
        if not CAN_AVAILABLE:
            self._log("❌ python-can がインストールされていません")
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
                        self._process_10ms_tx()
                    except Exception as e:
                        pass
                next_time_10ms += 0.010
            
            if now >= next_time_100ms:
                if self.bus:
                    try:
                        self._process_100ms_tx()
                    except Exception as e:
                        pass
                next_time_100ms += 0.100

            time.sleep(0.001)

    def _process_10ms_tx(self):
        with self.lock:
            for m in self.modules:
                if not m.get("tx_enabled", False):
                    continue
                
                base_id = m.get("baseId", 0)
                if m["type"] == "mdd":
                    state = m.setdefault("state", {"appMode": 0, "paramSendRequested": False})
                    if state["appMode"] == APP_MODE_PARAMETER:
                        if state["paramSendRequested"]:
                            for i in range(4): self._send_mdd_param(m, i)
                            self._send_mdd_mode(m)
                    else:
                        self._send_mdd_target(m)
                
                elif m["type"] == "servo":
                    self._send_servo_target(m)

    def _process_100ms_tx(self):
        with self.lock:
            for m in self.modules:
                if not m.get("tx_enabled", False):
                    continue
                if m["type"] == "solenoid":
                    self._send_solenoid_target(m)

    @staticmethod
    def _encode_i16_le(value: int):
        value &= 0xFFFF
        return value & 0xFF, (value >> 8) & 0xFF

    def _send_mdd_mode(self, m):
        payload = bytearray(4)
        for i in range(4): payload[i] = m["motors"][i]["mode"] & 0xFF
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=m["baseId"] + 0x10, data=payload))

    def _send_mdd_target(self, m):
        payload = bytearray(8)
        for i in range(4):
            t = int(round(float(m["motors"][i]["target"]) * 10.0))
            t = max(-32768, min(32767, t))
            low, high = self._encode_i16_le(t)
            payload[i*2] = low; payload[i*2+1] = high
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=m["baseId"] + 0x20, data=payload))

    def _send_mdd_param(self, m, i):
        motor = m["motors"][i]
        p_raw = int(round(float(motor["p"]) * 100.0))
        i_raw = int(round(float(motor["i"]) * 100.0))
        d_raw = int(round(float(motor["d"]) * 100.0))
        w_raw = int(round(float(motor["wheel"])))
        direction = 1 if motor["dir"] >= 0 else -1
        signed_w = w_raw * direction

        payload = bytearray(8)
        payload[0], payload[1] = self._encode_i16_le(p_raw)
        payload[2], payload[3] = self._encode_i16_le(i_raw)
        payload[4], payload[5] = self._encode_i16_le(d_raw)
        payload[6], payload[7] = self._encode_i16_le(signed_w)
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=m["baseId"] + i, data=payload))

    def _send_servo_target(self, m):
        payload = bytearray([max(0, min(180, int(v))) for v in m["ch"][:6]])
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=m["baseId"], data=payload))

    def _send_solenoid_target(self, m):
        v = int(m.get("valves", 0))
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=m["baseId"], data=[v & 0xFF, (v >> 8) & 0xFF]))

    # ---------------------------------------------------------
    # RX Loop & Trigger Evaluation
    # ---------------------------------------------------------
    def _rx_loop(self):
        while self.running:
            if self.bus:
                try:
                    msg = self.bus.recv(timeout=0.005)
                    while msg is not None:
                        self._process_rx_msg(msg)
                        msg = self.bus.recv(timeout=0.0)
                except Exception:
                    pass

    def _process_rx_msg(self, msg):
        with self.lock:
            for m in self.modules:
                if m["type"] == "mdd" and msg.arbitration_id == (m["baseId"] + 0x30):
                    if len(msg.data) >= 6:
                        state = m.setdefault("state", {})
                        state["sw"] = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
                        state["err"] = msg.data[4]
                        
                        prev_mode = state.get("appMode", 0)
                        new_mode = msg.data[5] & 0x01
                        state["appMode"] = new_mode
                        
                        if prev_mode == 0 and new_mode == 1:
                            state["paramSendRequested"] = False
                            self._log(f"✅ [{m['name']}] 制御実行モードへ移行")
                        elif prev_mode == 1 and new_mode == 0:
                            state["paramSendRequested"] = False
                            m["tx_enabled"] = False
                            self._log(f"⛔ [{m['name']}] パラメータ設定モードへ戻りました")
                        
                        # トリガーエンジンの評価
                        if self.auto_trigger_engine_enabled:
                            self._evaluate_triggers()

    def _evaluate_triggers(self):
        # 簡易的なトリガー評価（LOCK内から呼ばれる）
        for rule in self.automation_rules:
            if rule["trigger"] != "CONDITION":
                continue
            
            # check condition
            c_mod_id = rule.get("cond_module")
            c_var = rule.get("cond_var")
            c_val = rule.get("cond_val")
            
            c_mod = next((x for x in self.modules if x["id"] == c_mod_id), None)
            if not c_mod or c_mod["type"] != "mdd":
                continue
            
            try:
                sw_idx = int(c_var.replace("sw", "")) - 1
                current_val = c_mod["state"]["sw"][sw_idx]
                if current_val == int(c_val):
                    self._execute_actions(rule["actions"])
            except:
                pass

    def run_macro(self, rule):
        with self.lock:
            self._execute_actions(rule.get("actions", []))
            self._log(f"▶️ マクロ '{rule.get('name', 'Macro')}' を実行しました")

    def _execute_actions(self, actions):
        # LOCK済みを想定
        for act in actions:
            t_mod = next((x for x in self.modules if x["id"] == act.get("module")), None)
            if not t_mod: continue
            
            act_type = act.get("action")
            try:
                if t_mod["type"] == "mdd" and act_type == "set_target":
                    idx = int(act["idx"])
                    t_mod["motors"][idx]["target"] = float(act["val"])
                elif t_mod["type"] == "servo" and act_type == "set_angle":
                    idx = int(act["idx"])
                    t_mod["ch"][idx] = int(act["val"])
                elif t_mod["type"] == "solenoid" and act_type == "set_valve":
                    idx = int(act["idx"])
                    state = bool(int(act["val"]))
                    if state:
                        t_mod["valves"] |= (1 << idx)
                    else:
                        t_mod["valves"] &= ~(1 << idx)
            except:
                pass

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{ts}] {msg}")


# =============================================================================
# GUI アプリケーション
# =============================================================================
class AltairAdvancedGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Altair Module System Advanced GUI")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        
        self.log_queue = queue.Queue()
        self.modules = []
        self.automation_rules = []
        self.backend = None
        
        self.module_tabs = {} # id -> tab_frame
        self.mdd_ui_vars = {}
        self.servo_ui_vars = {}
        self.solenoid_ui_vars = {}
        
        self._load_config()
        self._build_ui()
        self._poll_log()
        self._poll_ui_update()

    def _load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.modules = data.get("modules", [])
                    self.automation_rules = data.get("automation_rules", [])
            except:
                pass

    def _save_config(self):
        try:
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump({
                    "modules": self.modules,
                    "automation_rules": self.automation_rules
                }, f, indent=2)
            self._log("System", "設定を保存しました", "info")
        except Exception as e:
            self._log("System", f"設定保存失敗: {e}", "danger")

    def _log(self, source, msg, level="info"):
        self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [{source}] {msg}")

    # ---------------------------------------------------------
    # UI構築
    # ---------------------------------------------------------
    def _build_ui(self):
        # -- 接続パネル --
        conn_frame = ttk.LabelFrame(self.root, text="CAN 接続", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Interface:").grid(row=0, column=0, sticky=tk.W)
        self.var_interface = tk.StringVar(value="socketcan")
        ttk.Combobox(conn_frame, textvariable=self.var_interface, values=("socketcan", "slcan", "pcan")).grid(row=0, column=1, padx=5)
        
        ttk.Label(conn_frame, text="Channel:").grid(row=0, column=2, sticky=tk.W, padx=(10, 0))
        self.var_channel = tk.StringVar(value="can0")
        ttk.Entry(conn_frame, textvariable=self.var_channel, width=15).grid(row=0, column=3, sticky=tk.W, padx=5)
        
        ttk.Label(conn_frame, text="Bitrate:").grid(row=0, column=4, sticky=tk.W, padx=(10, 0))
        self.var_bitrate = tk.StringVar(value="1000000")
        ttk.Combobox(conn_frame, textvariable=self.var_bitrate, values=["500000", "1000000"], width=10).grid(row=0, column=5, padx=5)
        
        self.btn_connect = ttk.Button(conn_frame, text="接続", command=self._on_connect)
        self.btn_connect.grid(row=0, column=6, padx=15)
        self.btn_disconnect = ttk.Button(conn_frame, text="切断", command=self._on_disconnect, state=tk.DISABLED)
        self.btn_disconnect.grid(row=0, column=7)

        # -- トップツールバー --
        tb = ttk.Frame(self.root)
        tb.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(tb, text="設定を保存", command=self._save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(tb, text="モジュールを追加", command=self._show_add_module_dialog).pack(side=tk.LEFT, padx=5)
        
        self.var_auto_trigger = tk.BooleanVar(value=False)
        cb_trigger = ttk.Checkbutton(tb, text="オートトリガーエンジン有効", variable=self.var_auto_trigger, command=self._on_trigger_toggle)
        cb_trigger.pack(side=tk.RIGHT, padx=5)

        # -- NOTEBOOK (タブ) --
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self._build_automation_tab()
        self._refresh_module_tabs()

        # -- ログビュー --
        log_frame = ttk.LabelFrame(self.root, text="システムログ", padding=5)
        log_frame.pack(fill=tk.X, padx=10, pady=5)
        self.txt_log = scrolledtext.ScrolledText(log_frame, height=8, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True)

    def _refresh_module_tabs(self):
        # Remove existing module tabs (keep automation at index 0)
        for tab_id in list(self.module_tabs.keys()):
            try:
                self.notebook.forget(self.module_tabs[tab_id])
            except: pass
            del self.module_tabs[tab_id]
        
        for m in self.modules:
            self._build_module_tab(m)

    def _show_add_module_dialog(self):
        win = tk.Toplevel(self.root)
        win.title("Add Module")
        win.geometry("300x200")
        
        ttk.Label(win, text="Type:").pack(pady=5)
        var_type = tk.StringVar(value="mdd")
        ttk.Combobox(win, textvariable=var_type, values=["mdd", "servo", "solenoid"]).pack()
        
        ttk.Label(win, text="Name:").pack(pady=5)
        var_name = tk.StringVar(value="New_Module")
        ttk.Entry(win, textvariable=var_name).pack()
        
        ttk.Label(win, text="Base CAN ID (Hex):").pack(pady=5)
        var_id = tk.StringVar(value="200")
        ttk.Entry(win, textvariable=var_id).pack()
        
        def _add():
            try:
                t = var_type.get()
                n = var_name.get()
                b = int(var_id.get(), 16)
                m = {"id": f"mod_{int(time.time()*1000)}", "type": t, "name": n, "baseId": b, "tx_enabled": False}
                if t == "mdd":
                    m["motors"] = [{"target":0, "mode":0, "p":80, "i":0, "d":2, "wheel":65, "dir":1} for _ in range(4)]
                    m["state"] = {"appMode":0, "paramSendRequested":False, "sw":[0,0,0,0], "err":0}
                elif t == "servo":
                    m["ch"] = [90]*6
                elif t == "solenoid":
                    m["valves"] = 0
                
                self.modules.append(m)
                self._refresh_module_tabs()
                if self.backend: self.backend.modules = self.modules
                self._refresh_automation_list()
                win.destroy()
            except:
                messagebox.showerror("Error", "Invalid inputs")
                
        ttk.Button(win, text="Add", command=_add).pack(pady=15)

    def _build_module_tab(self, m):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text=f" {m['name']} (0x{m['baseId']:X}) ")
        self.module_tabs[m['id']] = tab
        
        # 削除・TXオンオフ
        top = ttk.Frame(tab)
        top.pack(fill=tk.X, pady=5)
        
        def _del():
            if messagebox.askyesno("Confirm", "Delete this module?"):
                self.modules = [x for x in self.modules if x['id'] != m['id']]
                self._refresh_module_tabs()
                self._refresh_automation_list()
        
        ttk.Button(top, text="Delete Module", command=_del).pack(side=tk.RIGHT)
        
        var_tx = tk.BooleanVar(value=m.get("tx_enabled", False))
        def _tx_toggle(*args): m["tx_enabled"] = var_tx.get()
        var_tx.trace_add("write", _tx_toggle)
        ttk.Checkbutton(top, text="Enable TX (10ms/100ms Loop)", variable=var_tx).pack(side=tk.LEFT)

        if m["type"] == "mdd":
            self._build_mdd_ui(tab, m)
        elif m["type"] == "servo":
            self._build_servo_ui(tab, m)
        elif m["type"] == "solenoid":
            self._build_solenoid_ui(tab, m)

    def _build_mdd_ui(self, parent, m):
        f = ttk.LabelFrame(parent, text="Control & Params", padding=10)
        f.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        def _req_param():
            m.setdefault("state", {})["paramSendRequested"] = True
            
        ttk.Button(f, text="Send Parameters (Set Param Mode)", command=_req_param).pack(fill=tk.X, pady=5)
        
        st_label = ttk.Label(f, text="State: Offline", font=("", 10, "bold"))
        st_label.pack(pady=5)
        self.mdd_ui_vars[m['id']] = {"status_label": st_label, "motor_vars": []}

        for i in range(4):
            row = ttk.Frame(f)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"M{i+1} Tgt:").pack(side=tk.LEFT)
            
            vt = tk.DoubleVar(value=m["motors"][i]["target"])
            def _change_t(event=None, idx=i, var=vt): m["motors"][idx]["target"] = var.get()
            
            s = ttk.Scale(row, from_=-32767, to=32767, variable=vt, orient=tk.HORIZONTAL, command=_change_t)
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            e = ttk.Entry(row, textvariable=vt, width=8)
            e.pack(side=tk.LEFT)
            e.bind("<Return>", _change_t)
            
            self.mdd_ui_vars[m['id']]["motor_vars"].append(vt)

    def _build_servo_ui(self, parent, m):
        f = ttk.LabelFrame(parent, text="Servo Control", padding=10)
        f.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.servo_ui_vars[m['id']] = []
        
        for i in range(6):
            row = ttk.Frame(f)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"CH{i+1}:").pack(side=tk.LEFT)
            
            vt = tk.IntVar(value=m["ch"][i])
            def _change(event=None, idx=i, var=vt): m["ch"][idx] = var.get()
            
            s = ttk.Scale(row, from_=0, to=180, variable=vt, orient=tk.HORIZONTAL, command=_change)
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.servo_ui_vars[m['id']].append(vt)

    def _build_solenoid_ui(self, parent, m):
        f = ttk.LabelFrame(parent, text="Valve Control", padding=10)
        f.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.solenoid_ui_vars[m['id']] = []
        
        grid = ttk.Frame(f)
        grid.pack()
        
        for i in range(12):
            var = tk.BooleanVar(value=bool(m.get("valves",0) & (1<<i)))
            def _change(event=None, idx=i, v=var):
                if v.get(): m["valves"] |= (1<<idx)
                else: m["valves"] &= ~(1<<idx)
            
            cb = ttk.Checkbutton(grid, text=f"V{i+1}", variable=var, command=_change)
            cb.grid(row=i//4, column=i%4, padx=10, pady=10)
            self.solenoid_ui_vars[m['id']].append(var)

    # ---------------------------------------------------------
    # オートメーション UI
    # ---------------------------------------------------------
    def _build_automation_tab(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text=" ⚡ Automation ")
        
        top = ttk.Frame(tab)
        top.pack(fill=tk.X, pady=5)
        ttk.Button(top, text="Add Macro (Manual Run)", command=lambda: self._add_rule("MACRO")).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="Add Trigger (Condition Loop)", command=lambda: self._add_rule("CONDITION")).pack(side=tk.LEFT, padx=5)
        
        self.rule_list_frame = ttk.Frame(tab)
        self.rule_list_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self._refresh_automation_list()

    def _add_rule(self, type_str):
        self.automation_rules.append({
            "id": f"rule_{int(time.time()*1000)}",
            "name": f"New {type_str}",
            "trigger": type_str,
            "cond_module": "",
            "cond_var": "sw1",
            "cond_val": 1,
            "actions": []
        })
        self._refresh_automation_list()

    def _refresh_automation_list(self):
        for widget in self.rule_list_frame.winfo_children():
            widget.destroy()
            
        for rule in self.automation_rules:
            f = ttk.LabelFrame(self.rule_list_frame, text=f"[{rule['trigger']}] {rule['name']}", padding=5)
            f.pack(fill=tk.X, pady=5, padx=5)
            
            # Edit actions
            btn_f = ttk.Frame(f)
            btn_f.pack(side=tk.RIGHT)
            
            if rule["trigger"] == "MACRO":
                ttk.Button(btn_f, text="▶ Run Macro", command=lambda r=rule: self.backend.run_macro(r) if self.backend else None).pack(pady=2)
            ttk.Button(btn_f, text="Delete", command=lambda r=rule: self._delete_rule(r)).pack(pady=2)
            
            if rule["trigger"] == "CONDITION":
                c_f = ttk.Frame(f)
                c_f.pack(fill=tk.X)
                ttk.Label(c_f, text="IF MDD:").pack(side=tk.LEFT)
                mdd_opts = [m['id'] for m in self.modules if m['type']=='mdd']
                
                v_mod = tk.StringVar(value=rule.get("cond_module"))
                def _set_c_mod(e, r=rule, v=v_mod): r["cond_module"] = v.get()
                cb = ttk.Combobox(c_f, textvariable=v_mod, values=mdd_opts, width=15)
                cb.pack(side=tk.LEFT, padx=5)
                cb.bind("<<ComboboxSelected>>", _set_c_mod)
                
                ttk.Label(c_f, text="SW:").pack(side=tk.LEFT)
                v_var = tk.StringVar(value=rule.get("cond_var"))
                def _set_c_var(e, r=rule, v=v_var): r["cond_var"] = v.get()
                cb_var = ttk.Combobox(c_f, textvariable=v_var, values=["sw1","sw2","sw3","sw4"], width=5)
                cb_var.pack(side=tk.LEFT, padx=5)
                cb_var.bind("<<ComboboxSelected>>", _set_c_var)
                
                ttk.Label(c_f, text="==").pack(side=tk.LEFT)
                v_val = tk.StringVar(value=str(rule.get("cond_val")))
                def _set_c_val(e, r=rule, v=v_val): r["cond_val"] = int(v.get())
                cb_val = ttk.Combobox(c_f, textvariable=v_val, values=["1", "0"], width=3)
                cb_val.pack(side=tk.LEFT, padx=5)
                cb_val.bind("<<ComboboxSelected>>", _set_c_val)

            # Actions overview (JSON text for simplicity)
            ttk.Label(f, text=f"Actions: {json.dumps(rule['actions'])}").pack(anchor=tk.W, pady=5)
            
            # Action Editor (Simplistic)
            ttk.Button(f, text="Add Action (JSON)", command=lambda r=rule: self._prompt_add_action(r)).pack(anchor=tk.W)

    def _delete_rule(self, r):
        self.automation_rules.remove(r)
        self._refresh_automation_list()
        
    def _prompt_add_action(self, rule):
        ans = simpledialog.askstring("Add Action", "Format: module_id,set_target,idx,val\nExample: mod_123,set_valve,0,1")
        if ans:
            try:
                parts = ans.split(',')
                act = {"module": parts[0], "action": parts[1], "idx": int(parts[2]), "val": float(parts[3])}
                rule["actions"].append(act)
                self._refresh_automation_list()
            except:
                messagebox.showerror("Error", "Invalid format")


    # ---------------------------------------------------------
    # Core Actions
    # ---------------------------------------------------------
    def _on_connect(self):
        self.backend = AdvancedCANBackend(
            self.var_interface.get(),
            self.var_channel.get(),
            int(self.var_bitrate.get()),
            self.log_queue,
            self.modules,
            self.automation_rules
        )
        self.backend.auto_trigger_engine_enabled = self.var_auto_trigger.get()
        
        if self.backend.connect():
            self.backend.start()
            self.btn_connect.config(state=tk.DISABLED)
            self.btn_disconnect.config(state=tk.NORMAL)
        else:
            self.backend = None

    def _on_disconnect(self):
        if self.backend:
            self.backend.disconnect()
            self.backend = None
        self.btn_connect.config(state=tk.NORMAL)
        self.btn_disconnect.config(state=tk.DISABLED)

    def _on_trigger_toggle(self):
        if self.backend:
            self.backend.auto_trigger_engine_enabled = self.var_auto_trigger.get()
            self._log("System", f"Auto Trigger Engine: {'ON' if self.var_auto_trigger.get() else 'OFF'}")

    def _poll_log(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.txt_log.config(state=tk.NORMAL)
            self.txt_log.insert(tk.END, msg + "\n")
            self.txt_log.see(tk.END)
            self.txt_log.config(state=tk.DISABLED)
        self.root.after(100, self._poll_log)

    def _poll_ui_update(self):
        # Update MDD statuses
        for m in self.modules:
            if m["type"] == "mdd" and m["id"] in self.mdd_ui_vars:
                ui_obj = self.mdd_ui_vars[m["id"]]
                state = m.get("state", {})
                sw_str = state.get("sw", [0,0,0,0])
                mode_str = "CONTROL" if state.get("appMode") == 1 else "PARAM"
                ui_obj["status_label"].config(text=f"State: {mode_str} | SW: {sw_str} | Err: {state.get('err',0)}")
                
        self.root.after(200, self._poll_ui_update)

    def _on_close(self):
        if self.backend:
            self.backend.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = AltairAdvancedGUI(root)
    root.mainloop()
