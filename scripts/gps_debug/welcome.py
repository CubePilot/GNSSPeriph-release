"""Connection-settings dialog shown before the main GUI."""

import tkinter as tk
from tkinter import messagebox, ttk

from .utils import detect_serial_ports, use_hover_friendly_theme


class WelcomeDialog:
    """Modal-ish startup window that collects connection settings before launching the main GUI."""

    PRESETS = [
        ("Serial (USB) — pick from detected list", "mavcan:/dev/ttyACM0"),
        ("UDP listen (MAVProxy --out udpout)",     "mavcan:udpin:0.0.0.0:14550"),
        ("UDP connect to GCS",                     "mavcan:udpout:127.0.0.1:14550"),
        ("TCP to SITL / mavlink-router",           "mavcan:tcp:127.0.0.1:5760"),
    ]

    def __init__(self, defaults):
        self.result = None
        self.root = tk.Tk()
        use_hover_friendly_theme(self.root)
        self.root.title("Cube CAN GPS Debug — Connection")
        self.root.geometry("600x470")
        self.root.resizable(False, False)

        pad = {"padx": 8, "pady": 4}

        hdr = ttk.Frame(self.root, padding=10)
        hdr.pack(fill=tk.X)
        ttk.Label(hdr, text="Cube CAN GPS Debug",
                  font=("TkDefaultFont", 14, "bold")).pack(anchor=tk.W)
        ttk.Label(hdr, text="Configure MAVLink + DroneCAN-over-MAVLink before connecting.",
                  foreground="#555").pack(anchor=tk.W)

        body = ttk.Frame(self.root, padding=10)
        body.pack(fill=tk.BOTH, expand=True)

        # Preset selector — pick the entry that best matches whatever default
        # port we were handed (so dropdown stays consistent with Port URL).
        ttk.Label(body, text="Quick preset:").grid(row=0, column=0, sticky=tk.W, **pad)
        self.preset_var = tk.StringVar(value=self._guess_preset(defaults.get("port")))
        preset = ttk.Combobox(body, textvariable=self.preset_var, state="readonly",
                              values=[p[0] for p in self.PRESETS], width=42)
        preset.grid(row=0, column=1, columnspan=2, sticky=tk.EW, **pad)
        preset.bind("<<ComboboxSelected>>", self._on_preset)

        ttk.Label(body, text="Port URL:").grid(row=1, column=0, sticky=tk.W, **pad)
        self.port_var = tk.StringVar(value=defaults.get("port") or "")
        ttk.Entry(body, textvariable=self.port_var, width=46).grid(
            row=1, column=1, columnspan=2, sticky=tk.EW, **pad)
        ttk.Label(body,
                  text="e.g. mavcan:udpin:0.0.0.0:14550, mavcan:tcp:host:5760, mavcan:/dev/ttyACM0",
                  foreground="#777", font=("TkDefaultFont", 9)).grid(
            row=2, column=1, columnspan=2, sticky=tk.W, padx=8)

        # Detected serial ports
        ttk.Label(body, text="Detected serial:").grid(row=3, column=0, sticky=tk.W, **pad)
        serial_row = ttk.Frame(body)
        serial_row.grid(row=3, column=1, columnspan=2, sticky=tk.EW, **pad)
        self.serial_var = tk.StringVar(value="")
        self.serial_cbo = ttk.Combobox(serial_row, textvariable=self.serial_var,
                                       state="readonly", width=42)
        self.serial_cbo.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.serial_cbo.bind("<<ComboboxSelected>>", self._on_serial_pick)
        ttk.Button(serial_row, text="Rescan", width=8,
                   command=self._refresh_serial).pack(side=tk.LEFT, padx=(6, 0))
        self._serial_devices = []
        self._refresh_serial()

        ttk.Label(body, text="Serial baud:").grid(row=4, column=0, sticky=tk.W, **pad)
        self.baud_var = tk.StringVar(value=str(defaults.get("baudrate", 115200)))
        ttk.Combobox(body, textvariable=self.baud_var, width=12,
                     values=["57600", "115200", "230400", "460800", "921600", "1500000"]).grid(
            row=4, column=1, sticky=tk.W, **pad)
        ttk.Label(body, text="(ignored for udp/tcp)", foreground="#777").grid(
            row=4, column=2, sticky=tk.W, **pad)

        ttk.Label(body, text="CAN bus #:").grid(row=5, column=0, sticky=tk.W, **pad)
        self.bus_var = tk.StringVar(value=str(defaults.get("bus", 1)))
        ttk.Spinbox(body, from_=1, to=3, textvariable=self.bus_var, width=6).grid(
            row=5, column=1, sticky=tk.W, **pad)
        ttk.Label(body, text="(autopilot bus carrying the GPS)", foreground="#777").grid(
            row=5, column=2, sticky=tk.W, **pad)

        ttk.Label(body, text="Local node ID:").grid(row=6, column=0, sticky=tk.W, **pad)
        self.node_id_var = tk.StringVar(value=str(defaults.get("node_id", 125)))
        ttk.Spinbox(body, from_=1, to=125, textvariable=self.node_id_var, width=6).grid(
            row=6, column=1, sticky=tk.W, **pad)
        ttk.Label(body, text="(this script's DroneCAN node id, must be unique)",
                  foreground="#777").grid(row=6, column=2, sticky=tk.W, **pad)

        ttk.Label(body, text="MAVLink target sys:").grid(row=7, column=0, sticky=tk.W, **pad)
        self.target_sys_var = tk.StringVar(value=str(defaults.get("target_system", 0)))
        ttk.Spinbox(body, from_=0, to=255, textvariable=self.target_sys_var, width=6).grid(
            row=7, column=1, sticky=tk.W, **pad)
        ttk.Label(body, text="(0 = first system seen)", foreground="#777").grid(
            row=7, column=2, sticky=tk.W, **pad)

        ttk.Label(body, text="MAVLink2 signing key:").grid(row=8, column=0, sticky=tk.W, **pad)
        self.signing_var = tk.StringVar(value=defaults.get("signing_key") or "")
        ttk.Entry(body, textvariable=self.signing_var, width=46, show="*").grid(
            row=8, column=1, columnspan=2, sticky=tk.EW, **pad)

        body.columnconfigure(1, weight=1)

        btns = ttk.Frame(self.root, padding=10)
        btns.pack(fill=tk.X)
        ttk.Button(btns, text="Cancel", command=self._on_cancel).pack(side=tk.RIGHT)
        ttk.Button(btns, text="Connect", command=self._on_connect).pack(side=tk.RIGHT, padx=6)

        self.root.bind("<Return>", lambda _e: self._on_connect())
        self.root.protocol("WM_DELETE_WINDOW", self._on_cancel)

    # ---------------------- behaviour ----------------------
    def _guess_preset(self, port):
        serial_name = next(name for name, _ in self.PRESETS if "Serial" in name)
        if not port:
            return serial_name
        for name, url in self.PRESETS:
            if "Serial" in name and (port.startswith("mavcan:/dev/")
                                     or port.startswith("mavcan:com")
                                     or port.startswith("mavcan:COM")):
                return name
            if "udpin"  in url and "udpin"  in port: return name
            if "udpout" in url and "udpout" in port: return name
            if "tcp"    in url and "tcp"    in port: return name
        return serial_name

    def _on_preset(self, _evt=None):
        for name, url in self.PRESETS:
            if name == self.preset_var.get():
                if "Serial" in name and self._serial_devices:
                    self.port_var.set("mavcan:" + self._serial_devices[0][0])
                    self.serial_var.set(self._format_serial_choice(self._serial_devices[0]))
                else:
                    self.port_var.set(url)
                return

    def _refresh_serial(self):
        ports = detect_serial_ports()
        self._serial_devices = ports
        labels = [self._format_serial_choice(p) for p in ports]
        self.serial_cbo["values"] = labels
        if not ports:
            self.serial_cbo.set("(no serial ports detected)")
            return
        # Prefer a port whose description looks like an autopilot.
        preferred = next((i for i, (_d, desc) in enumerate(ports)
                          if any(k in desc.lower()
                                 for k in ("cube", "ardupilot", "pixhawk", "px4"))), 0)
        self.serial_cbo.current(preferred)
        cur = self.port_var.get().strip()
        placeholders = ("", "mavcan:/dev/ttyACM0",
                        *(url for _n, url in self.PRESETS if "Serial" in _n))
        if cur in placeholders:
            self.port_var.set("mavcan:" + ports[preferred][0])

    def _on_serial_pick(self, _evt=None):
        idx = self.serial_cbo.current()
        if idx < 0 or idx >= len(self._serial_devices):
            return
        device, _desc = self._serial_devices[idx]
        self.port_var.set("mavcan:" + device)
        for name, _url in self.PRESETS:
            if "Serial" in name:
                self.preset_var.set(name)
                break

    @staticmethod
    def _format_serial_choice(p):
        device, desc = p
        return f"{device}  —  {desc}" if desc else device

    def _on_connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Missing port", "Port URL is required.")
            return
        try:
            cfg = dict(
                port=port,
                node_id=int(self.node_id_var.get()),
                bus=int(self.bus_var.get()),
                target_system=int(self.target_sys_var.get()),
                baudrate=int(self.baud_var.get()),
                signing_key=self.signing_var.get().strip() or None,
            )
        except ValueError as ex:
            messagebox.showerror("Invalid input", str(ex))
            return
        self.result = cfg
        self.root.destroy()

    def _on_cancel(self):
        self.result = None
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        return self.result
