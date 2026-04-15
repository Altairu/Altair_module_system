import queue
import struct
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import can


CAN_BITRATE = 1000000
PARAM_CAN_IDS = [0x200, 0x201, 0x203, 0x204]


class AltairCanToolUbuntu:
    def __init__(self, root):
        self.root = root
        self.root.title("Altair MDD Controller (Ubuntu)")
        self.root.geometry("1220x840")
        self.root.minsize(1120, 720)
        self.root.configure(bg="#0B1220")

        self.bus = None
        self.rx_running = False
        self.rx_queue = queue.Queue()

        self.iface_var = tk.StringVar(value="socketcan")
        self.chan_var = tk.StringVar(value="can0")
        self.mode_var = tk.IntVar(value=0)

        self.motor_param_vars = {}
        self.target_vars = []

        self.limit_vars = [tk.StringVar(value="-") for _ in range(4)]
        self.status_mode_vars = [tk.StringVar(value="-") for _ in range(4)]
        self.error_var = tk.StringVar(value="0x00 (NORMAL)")
        self.state_var = tk.StringVar(value="-")
        self.last_rx_var = tk.StringVar(value="No data yet")

        self._setup_style()
        self._build_ui()
        self.root.after(100, self._process_rx_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_style(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#0B1220")
        style.configure("TLabelframe", background="#111A2C", foreground="#DDE6F6", borderwidth=1)
        style.configure("TLabelframe.Label", background="#111A2C", foreground="#8EC5FF", font=("DejaVu Sans", 10, "bold"))
        style.configure("TLabel", background="#111A2C", foreground="#DDE6F6", font=("DejaVu Sans", 10))
        style.configure("Header.TLabel", background="#0B1220", foreground="#DDE6F6", font=("DejaVu Sans", 16, "bold"))
        style.configure("Sub.TLabel", background="#0B1220", foreground="#8BA3C7", font=("DejaVu Sans", 10))
        style.configure("TButton", font=("DejaVu Sans", 10, "bold"), padding=6)
        style.configure("Accent.TButton", background="#0EA5A5", foreground="#061019")
        style.map("Accent.TButton", background=[("active", "#22D3EE")])
        style.configure("Warn.TButton", background="#FB923C", foreground="#1F1304")
        style.map("Warn.TButton", background=[("active", "#FDBA74")])

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        ttk.Label(main, text="Altair MDD CAN Control Panel", style="Header.TLabel").pack(anchor="w")
        ttk.Label(
            main,
            text="Ubuntu USB-to-CAN / SocketCAN | 1 Mbps target | 4 motors full control",
            style="Sub.TLabel",
        ).pack(anchor="w", pady=(0, 10))

        self._build_connection_panel(main)
        self._build_param_panel(main)
        self._build_control_panel(main)
        self._build_status_panel(main)
        self._build_log_panel(main)

    def _build_connection_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="CAN Connection (Ubuntu)")
        frame.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, text="Interface").grid(row=0, column=0, padx=8, pady=8)
        ttk.Combobox(
            frame,
            textvariable=self.iface_var,
            values=["socketcan", "slcan", "pcan", "kvaser"],
            state="readonly",
            width=12,
        ).grid(row=0, column=1, padx=4)

        ttk.Label(frame, text="Channel").grid(row=0, column=2, padx=8)
        ttk.Entry(frame, textvariable=self.chan_var, width=12).grid(row=0, column=3, padx=4)

        ttk.Label(frame, text="Target bitrate: 1,000,000 bps").grid(row=0, column=4, padx=10)

        self.btn_connect = ttk.Button(frame, text="Connect", style="Accent.TButton", command=self.connect_can)
        self.btn_connect.grid(row=0, column=5, padx=(10, 4), pady=8)
        ttk.Button(frame, text="Disconnect", command=self.disconnect_can).grid(row=0, column=6, padx=4)

        self.conn_status = ttk.Label(frame, text="Not connected", foreground="#F87171")
        self.conn_status.grid(row=0, column=7, padx=12)

        ttk.Label(
            frame,
            text="For socketcan: sudo ip link set can0 up type can bitrate 1000000",
            foreground="#8BA3C7",
        ).grid(row=1, column=0, columnspan=8, padx=8, pady=(0, 8), sticky="w")

    def _build_param_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Per-Motor PID Parameters (Gain x100, Dia mm)")
        frame.pack(fill="x", pady=(0, 10))

        headers = ["Motor", "CAN ID", "P", "I", "D", "Dia", "Invert", "Send"]
        for col, title in enumerate(headers):
            ttk.Label(frame, text=title, foreground="#93C5FD").grid(row=0, column=col, padx=6, pady=(8, 4), sticky="w")

        for idx, can_id in enumerate(PARAM_CAN_IDS):
            row = idx + 1
            p_var = tk.DoubleVar(value=1.50)
            i_var = tk.DoubleVar(value=0.10)
            d_var = tk.DoubleVar(value=0.05)
            dia_var = tk.IntVar(value=100)
            inv_var = tk.BooleanVar(value=False)

            self.motor_param_vars[idx] = {
                "can_id": can_id,
                "p": p_var,
                "i": i_var,
                "d": d_var,
                "dia": dia_var,
                "inv": inv_var,
            }

            ttk.Label(frame, text=f"M{idx + 1}").grid(row=row, column=0, padx=6, pady=4, sticky="w")
            ttk.Label(frame, text=f"0x{can_id:03X}").grid(row=row, column=1, padx=6, pady=4, sticky="w")
            ttk.Entry(frame, textvariable=p_var, width=8).grid(row=row, column=2, padx=6)
            ttk.Entry(frame, textvariable=i_var, width=8).grid(row=row, column=3, padx=6)
            ttk.Entry(frame, textvariable=d_var, width=8).grid(row=row, column=4, padx=6)
            ttk.Entry(frame, textvariable=dia_var, width=8).grid(row=row, column=5, padx=6)
            ttk.Checkbutton(frame, variable=inv_var).grid(row=row, column=6, padx=6)
            ttk.Button(frame, text=f"Send M{idx + 1}", command=lambda i=idx: self.send_param_single(i)).grid(
                row=row, column=7, padx=6
            )

        btn_row = len(PARAM_CAN_IDS) + 1
        ttk.Button(frame, text="Copy M1 to All", command=self.copy_m1_to_all).grid(row=btn_row, column=0, columnspan=3, pady=8)
        ttk.Button(frame, text="Send All Params", style="Accent.TButton", command=self.send_params_all).grid(
            row=btn_row, column=3, columnspan=3, pady=8
        )

    def _build_control_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Control Mode and Targets (x10)")
        frame.pack(fill="x", pady=(0, 10))

        ttk.Radiobutton(frame, text="Velocity", variable=self.mode_var, value=0).grid(row=0, column=0, padx=6, pady=8)
        ttk.Radiobutton(frame, text="Angle / Position", variable=self.mode_var, value=1).grid(row=0, column=1, padx=6, pady=8)
        ttk.Button(frame, text="Send Mode", command=self.send_mode).grid(row=0, column=2, padx=8, pady=8)
        ttk.Button(frame, text="STOP ALL", style="Warn.TButton", command=self.stop_motors).grid(row=0, column=3, padx=8, pady=8)

        for i in range(4):
            ttk.Label(frame, text=f"M{i + 1} Target").grid(row=1, column=i * 2, padx=6, pady=(4, 8), sticky="w")
            var = tk.DoubleVar(value=0.0)
            self.target_vars.append(var)
            ttk.Entry(frame, textvariable=var, width=10).grid(row=1, column=i * 2 + 1, padx=6, pady=(4, 8))

        ttk.Button(frame, text="Send Targets", style="Accent.TButton", command=self.send_targets).grid(
            row=2, column=0, columnspan=2, padx=8, pady=(0, 8), sticky="w"
        )

    def _build_status_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Status (from 0x120 / 0x121)")
        frame.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, text="Limit SW").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        for i in range(4):
            ttk.Label(frame, text=f"SW{i + 1}").grid(row=0, column=i + 1, padx=6, pady=6)
            ttk.Label(frame, textvariable=self.limit_vars[i], foreground="#67E8F9").grid(row=1, column=i + 1, padx=6)

        ttk.Label(frame, text="Motor Mode").grid(row=2, column=0, padx=6, pady=(8, 4), sticky="w")
        for i in range(4):
            ttk.Label(frame, text=f"M{i + 1}").grid(row=2, column=i + 1, padx=6)
            ttk.Label(frame, textvariable=self.status_mode_vars[i], foreground="#A7F3D0").grid(row=3, column=i + 1, padx=6)

        ttk.Label(frame, text="Error").grid(row=4, column=0, padx=6, pady=(8, 4), sticky="w")
        ttk.Label(frame, textvariable=self.error_var, foreground="#FDBA74").grid(row=4, column=1, columnspan=2, sticky="w")
        ttk.Label(frame, text="System State").grid(row=4, column=3, padx=6, sticky="e")
        ttk.Label(frame, textvariable=self.state_var, foreground="#93C5FD").grid(row=4, column=4, sticky="w")
        ttk.Label(frame, text="Last RX").grid(row=5, column=0, padx=6, pady=(4, 8), sticky="w")
        ttk.Label(frame, textvariable=self.last_rx_var).grid(row=5, column=1, columnspan=4, sticky="w")

    def _build_log_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="CAN Log")
        frame.pack(fill="both", expand=True)

        self.log_text = tk.Text(
            frame,
            height=12,
            state="disabled",
            bg="#070B14",
            fg="#DDE6F6",
            insertbackground="#DDE6F6",
            relief="flat",
            font=("DejaVu Sans Mono", 10),
        )
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _build_can_bus(self):
        iface = self.iface_var.get().strip()
        channel = self.chan_var.get().strip()

        if iface in ("socketcan", "socketcan_native"):
            return can.interface.Bus(bustype=iface, channel=channel)

        return can.interface.Bus(bustype=iface, channel=channel, bitrate=CAN_BITRATE)

    def connect_can(self):
        if self.bus is not None:
            self.log("Already connected")
            return
        try:
            self.bus = self._build_can_bus()
            self.conn_status.config(text=f"Connected {self.iface_var.get()}:{self.chan_var.get()}", foreground="#22C55E")
            self.log(f"CONNECTED: {self.iface_var.get()} {self.chan_var.get()} @ target {CAN_BITRATE} bps")
            self.rx_running = True
            threading.Thread(target=self.rx_thread, daemon=True).start()
        except Exception as exc:
            self.bus = None
            self.conn_status.config(text="Connection failed", foreground="#F87171")
            self.log(f"CONNECT ERROR: {exc}")
            messagebox.showerror("Connection Error", str(exc))

    def disconnect_can(self):
        self.rx_running = False
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.conn_status.config(text="Not connected", foreground="#F87171")
        self.log("DISCONNECTED")

    def send_can_msg(self, can_id, payload):
        if self.bus is None:
            messagebox.showwarning("CAN Error", "CAN bus is not connected")
            return False
        try:
            msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
            self.bus.send(msg)
            hex_data = " ".join(f"{b:02X}" for b in payload)
            self.log(f"TX 0x{can_id:03X}: [{hex_data}]")
            return True
        except Exception as exc:
            self.log(f"TX ERROR 0x{can_id:03X}: {exc}")
            return False

    def _pack_motor_param(self, idx):
        item = self.motor_param_vars[idx]
        p = int(item["p"].get() * 100)
        i = int(item["i"].get() * 100)
        d = int(item["d"].get() * 100)
        dia = int(item["dia"].get())
        if item["inv"].get():
            dia = -abs(dia)
        else:
            dia = abs(dia)
        return struct.pack("<hhhh", p, i, d, dia)

    def send_param_single(self, idx):
        try:
            can_id = self.motor_param_vars[idx]["can_id"]
            payload = self._pack_motor_param(idx)
            if self.send_can_msg(can_id, payload):
                self.log(f"PARAM SENT: M{idx + 1} (0x{can_id:03X})")
        except Exception as exc:
            messagebox.showerror("Parameter Error", f"M{idx + 1}: {exc}")

    def send_params_all(self):
        ok = 0
        for idx in range(4):
            try:
                can_id = self.motor_param_vars[idx]["can_id"]
                payload = self._pack_motor_param(idx)
                if self.send_can_msg(can_id, payload):
                    ok += 1
            except Exception as exc:
                self.log(f"PARAM ERROR M{idx + 1}: {exc}")
        self.log(f"PARAM RESULT: {ok}/4 sent")

    def copy_m1_to_all(self):
        base = self.motor_param_vars[0]
        for idx in range(1, 4):
            self.motor_param_vars[idx]["p"].set(base["p"].get())
            self.motor_param_vars[idx]["i"].set(base["i"].get())
            self.motor_param_vars[idx]["d"].set(base["d"].get())
            self.motor_param_vars[idx]["dia"].set(base["dia"].get())
            self.motor_param_vars[idx]["inv"].set(base["inv"].get())
        self.log("Copied M1 PID parameters to M2-M4")

    def send_mode(self):
        mode = int(self.mode_var.get())
        payload = struct.pack("<hhhh", mode, mode, mode, mode)
        if self.send_can_msg(0x211, payload):
            self.log(f"MODE SENT: {'Velocity' if mode == 0 else 'Angle'}")

    def send_targets(self):
        try:
            targets = [int(var.get() * 10) for var in self.target_vars]
            payload = struct.pack("<hhhh", targets[0], targets[1], targets[2], targets[3])
            if self.send_can_msg(0x210, payload):
                raw = " ".join(f"M{i + 1}={self.target_vars[i].get():.1f}" for i in range(4))
                self.log(f"TARGET SENT: {raw}")
        except Exception as exc:
            messagebox.showerror("Target Error", str(exc))

    def stop_motors(self):
        for var in self.target_vars:
            var.set(0.0)
        self.send_targets()

    def rx_thread(self):
        while self.rx_running:
            try:
                if self.bus is None:
                    break
                msg = self.bus.recv(timeout=0.2)
                if msg is not None:
                    self.rx_queue.put(msg)
            except Exception:
                pass

    def _process_rx_queue(self):
        while True:
            try:
                msg = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_rx_message(msg)
        self.root.after(100, self._process_rx_queue)

    def _handle_rx_message(self, msg):
        if msg.arbitration_id == 0x120 and len(msg.data) >= 4:
            for i in range(4):
                self.limit_vars[i].set(str(int(msg.data[i])))
            self.last_rx_var.set("0x120 Limit")
            self.log(f"RX 0x120: Limits {list(msg.data[:4])}")
            return

        if msg.arbitration_id == 0x121 and len(msg.data) >= 6:
            modes = [int(msg.data[i]) for i in range(4)]
            for i, mode in enumerate(modes):
                self.status_mode_vars[i].set("VEL" if mode == 0 else "ANG")

            err = int(msg.data[4])
            state = int(msg.data[5])
            self.error_var.set(f"0x{err:02X} ({self._decode_error(err)})")
            self.state_var.set("CONTROL" if state == 1 else "PARAM")
            self.last_rx_var.set("0x121 Status")
            self.log(f"RX 0x121: modes={modes} err=0x{err:02X} state={state}")

    def _decode_error(self, err):
        if err == 0:
            return "NORMAL"
        names = []
        if err & 0x01:
            names.append("INIT_TIMEOUT")
        if err & 0x02:
            names.append("CAN_RX_TIMEOUT")
        if err & 0x04:
            names.append("CAN_TX_FAIL")
        return "|".join(names) if names else "UNKNOWN"

    def _on_close(self):
        self.disconnect_can()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    AltairCanToolUbuntu(root)
    root.mainloop()
