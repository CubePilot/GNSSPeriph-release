"""Main GUI window: nodes table, GPS telemetry, options, tunnel, log."""

import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from .constants import (
    FIX2_MODE,
    FIX2_STATUS,
    FIX2_SUBMODE,
    GPS_DRV_BIT_UBX_DEBUG,
)
from .tunnel import TunnelSession
from .utils import use_hover_friendly_theme


class GpsDebugGui:
    def __init__(self, root, worker):
        self.root = root
        self.worker = worker
        self.selected_node = None
        self.last_options_seen = None
        self.debug_var = None         # tk.IntVar for the UBX_DebugMessages checkbox
        self._suppress_write = False  # set when we update debug_var from a Read response
        self._await_fix_for_read = False  # arm an auto-read that fires after first Fix2 arrives

        use_hover_friendly_theme(root)
        root.title("Cube CAN GPS Debug")
        root.geometry("1100x720")

        # Each panel keeps its natural height; the log at the bottom expands to
        # fill leftover space. The Treeview has its own horizontal scrollbar
        # and the GPS / Options panels use _hscroll_frame for x-scroll on
        # narrow windows. (An outer canvas wrapper was tried but caused random
        # vertical drift as the log content repainted.)
        self.body = root

        self._build_topbar()
        self._build_nodes_table()
        self._build_gps_panel()
        self._build_options_panel()
        self._build_tunnel_panel()
        self._build_log()

        self.root.after(250, self._refresh_gui)

    # ---------------------- layout helpers ----------------------
    @staticmethod
    def _hscroll_frame(parent):
        """Pack a horizontally-scrollable inner frame into `parent`.

        Returns the inner ttk.Frame. The canvas auto-sizes its height to the
        inner content's natural height — no vertical scrolling, only an
        x-scrollbar appears when the inner content is wider than `parent`.
        """
        bg = ttk.Style().lookup("TFrame", "background") or "#d9d9d9"
        canvas = tk.Canvas(parent, highlightthickness=0, height=1, bg=bg)
        hsb = ttk.Scrollbar(parent, orient="horizontal", command=canvas.xview)
        canvas.configure(xscrollcommand=hsb.set)
        canvas.pack(fill=tk.X)
        hsb.pack(fill=tk.X)
        inner = ttk.Frame(canvas)
        canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_configure(_evt):
            canvas.configure(scrollregion=canvas.bbox("all"),
                             height=inner.winfo_reqheight())
        inner.bind("<Configure>", _on_inner_configure)
        return inner

    # ---------------------- panel builders ----------------------
    def _build_topbar(self):
        bar = ttk.Frame(self.body, padding=6)
        bar.pack(fill=tk.X)
        ttk.Label(bar, text=f"Port: {self.worker.port}").pack(side=tk.LEFT, padx=8)
        ttk.Label(bar, text=f"Bus: {self.worker.bus}").pack(side=tk.LEFT, padx=8)
        ttk.Label(bar, text=f"Local NodeID: {self.worker.node_id}").pack(side=tk.LEFT, padx=8)
        ttk.Label(bar, text=f"Target sys: {self.worker.target_system}").pack(side=tk.LEFT, padx=8)
        self.status_var = tk.StringVar(value="connecting…")
        ttk.Label(bar, textvariable=self.status_var, foreground="blue").pack(side=tk.RIGHT, padx=8)

    def _build_nodes_table(self):
        frm = ttk.LabelFrame(self.body, text="DroneCAN nodes", padding=4)
        frm.pack(fill=tk.X, padx=6, pady=4)
        cols = ("nid", "name", "hw", "sw", "mode", "health", "uptime", "vendor")
        self.tree = ttk.Treeview(frm, columns=cols, show="headings", height=6, selectmode="browse")
        widths = {"nid": 60, "name": 260, "hw": 70, "sw": 280, "mode": 90, "health": 80, "uptime": 80, "vendor": 80}
        labels = {"nid": "NodeID", "name": "Name", "hw": "HW", "sw": "SW (vcs/crc)", "mode": "Mode",
                  "health": "Health", "uptime": "Uptime", "vendor": "Vendor"}
        for c in cols:
            self.tree.heading(c, text=labels[c])
            # stretch=False keeps the column at its preferred width when the
            # parent shrinks, so the h-scrollbar reveals hidden columns
            # instead of squishing them down to two characters.
            self.tree.column(c, width=widths[c], anchor=tk.W, stretch=False)
        tree_xsb = ttk.Scrollbar(frm, orient="horizontal", command=self.tree.xview)
        self.tree.configure(xscrollcommand=tree_xsb.set)
        self.tree.pack(fill=tk.X)
        tree_xsb.pack(fill=tk.X)
        self.tree.bind("<<TreeviewSelect>>", self._on_node_select)

    def _build_gps_panel(self):
        frm = ttk.LabelFrame(self.body, text="GPS telemetry (selected node)", padding=6)
        frm.pack(fill=tk.X, padx=6, pady=4)
        inner = self._hscroll_frame(frm)
        self.gps_vars = {k: tk.StringVar(value="—") for k in (
            "fix", "sats", "lat", "lon", "alt_msl", "alt_ellip",
            "vel_n", "vel_e", "vel_d", "hdop", "vdop", "pdop", "age_fix2", "age_aux"
        )}
        rows = [
            [("Fix", "fix"), ("Sats used/visible", "sats"), ("HDOP", "hdop"), ("VDOP", "vdop"), ("PDOP", "pdop")],
            [("Lat (deg)", "lat"), ("Lon (deg)", "lon"), ("Alt MSL (m)", "alt_msl"), ("Alt ellip (m)", "alt_ellip")],
            [("VelN (m/s)", "vel_n"), ("VelE (m/s)", "vel_e"), ("VelD (m/s)", "vel_d"),
             ("Fix2 age (s)", "age_fix2"), ("Aux age (s)", "age_aux")],
        ]
        for r, row in enumerate(rows):
            for c, (label, key) in enumerate(row):
                ttk.Label(inner, text=label + ":").grid(row=r, column=c * 2, sticky=tk.W, padx=4, pady=2)
                ttk.Label(inner, textvariable=self.gps_vars[key], width=18, anchor=tk.W,
                          font=("TkFixedFont",)).grid(row=r, column=c * 2 + 1, sticky=tk.W, padx=4, pady=2)

    def _build_options_panel(self):
        frm = ttk.LabelFrame(self.body, text="Options (selected node)", padding=6)
        frm.pack(fill=tk.X, padx=6, pady=4)
        inner = self._hscroll_frame(frm)

        top = ttk.Frame(inner); top.pack(fill=tk.X)
        ttk.Label(top, text="Current value:").pack(side=tk.LEFT)
        self.opts_value_var = tk.StringVar(value="(select a node)")
        ttk.Label(top, textvariable=self.opts_value_var, font=("TkFixedFont",)).pack(side=tk.LEFT, padx=6)

        ttk.Button(top, text="Re-read", command=self._do_read).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Reboot node", command=self._do_reboot).pack(side=tk.LEFT, padx=4)

        toggle_row = ttk.Frame(inner); toggle_row.pack(fill=tk.X, pady=4)
        self.debug_var = tk.IntVar(value=0)
        ttk.Checkbutton(toggle_row,
                        text=f"UBX_DebugMessages (bit 0x{GPS_DRV_BIT_UBX_DEBUG:03x}) — auto-applied on toggle",
                        variable=self.debug_var,
                        command=self._on_debug_toggle).pack(side=tk.LEFT, padx=6)
        ttk.Label(toggle_row,
                  text="  (Reboot node to apply)",
                  foreground="#777").pack(side=tk.LEFT)

    def _build_tunnel_panel(self):
        frm = ttk.LabelFrame(self.body, text="Serial forward to u-blox (uavcan.tunnel.Targetted)", padding=6)
        frm.pack(fill=tk.X, padx=6, pady=4)
        inner = self._hscroll_frame(frm)

        row = ttk.Frame(inner); row.pack(fill=tk.X, pady=2)

        ttk.Label(row, text="TCP port:").pack(side=tk.LEFT, padx=(2, 2))
        self.tunnel_port_var = tk.StringVar(value="2001")
        ttk.Entry(row, textvariable=self.tunnel_port_var, width=7).pack(side=tk.LEFT)

        ttk.Label(row, text="Bind:").pack(side=tk.LEFT, padx=(8, 2))
        self.tunnel_host_var = tk.StringVar(value="127.0.0.1")
        ttk.Entry(row, textvariable=self.tunnel_host_var, width=14).pack(side=tk.LEFT)

        ttk.Label(row, text="Serial id:").pack(side=tk.LEFT, padx=(8, 2))
        self.tunnel_serial_var = tk.StringVar(value="-1")
        ttk.Combobox(row, textvariable=self.tunnel_serial_var, width=4,
                     values=["-1", "0", "1", "2", "3"]).pack(side=tk.LEFT)

        ttk.Label(row, text="Baud:").pack(side=tk.LEFT, padx=(8, 2))
        self.tunnel_baud_var = tk.StringVar(value="115200")
        ttk.Combobox(row, textvariable=self.tunnel_baud_var, width=8,
                     values=["9600", "38400", "57600", "115200", "230400",
                             "460800", "921600"]).pack(side=tk.LEFT)

        self.tunnel_lock_var = tk.IntVar(value=1)
        ttk.Checkbutton(row, text="Lock port (exclusive)",
                        variable=self.tunnel_lock_var).pack(side=tk.LEFT, padx=(10, 2))

        self.tunnel_start_btn = ttk.Button(row, text="Start", command=self._do_tunnel_start)
        self.tunnel_start_btn.pack(side=tk.LEFT, padx=(10, 2))
        self.tunnel_stop_btn = ttk.Button(row, text="Stop",
                                          command=self._do_tunnel_stop, state=tk.DISABLED)
        self.tunnel_stop_btn.pack(side=tk.LEFT, padx=2)

        ttk.Label(row, text="Status:").pack(side=tk.LEFT, padx=(10, 2))
        self.tunnel_status_var = tk.StringVar(value="stopped")
        ttk.Label(row, textvariable=self.tunnel_status_var,
                  font=("TkFixedFont",), foreground="#0066cc").pack(side=tk.LEFT)

        self.tunnel_bytes_var = tk.StringVar(value="")
        ttk.Label(row, textvariable=self.tunnel_bytes_var,
                  font=("TkFixedFont",), foreground="#666").pack(side=tk.LEFT, padx=(10, 2))

    def _build_log(self):
        frm = ttk.LabelFrame(self.body, text="Log", padding=4)
        frm.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        self.log = scrolledtext.ScrolledText(frm, height=10, state=tk.DISABLED, font=("TkFixedFont",))
        self.log.pack(fill=tk.BOTH, expand=True)

    # ---------------------- log routing ----------------------
    def log_line(self, line):
        # Called from worker thread — marshal to GUI thread.
        self.root.after(0, self._append_log, line)

    def _append_log(self, line):
        ts = time.strftime("%H:%M:%S")
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, f"[{ts}] {line}\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    # ---------------------- selection / actions ----------------------
    def _on_node_select(self, _evt):
        sel = self.tree.selection()
        if not sel:
            return
        nid = int(self.tree.item(sel[0], "values")[0])
        if nid == self.selected_node:
            return
        self.selected_node = nid
        self.last_options_seen = None
        self.opts_value_var.set("(waiting for GPS fix message…)")
        self._suppress_write = True
        try:
            self.debug_var.set(0)
        finally:
            self._suppress_write = False
        # Defer the GPS_DRV_OPTIONS read until a Fix2 arrives — confirms it
        # really is a GPS node before we hit it with a service request.
        self._await_fix_for_read = True

    def _require_selection(self):
        if self.selected_node is None:
            messagebox.showwarning("No node", "Select a node from the list first.")
            return False
        return True

    def _do_read(self):
        if not self._require_selection():
            return
        nid = self.selected_node
        self.opts_value_var.set("(reading…)")
        self.worker.post(lambda: self.worker.request_get_gps_drv_options(nid))

    def _on_debug_toggle(self):
        if self._suppress_write:
            return
        if not self._require_selection():
            return
        if self.last_options_seen is None:
            messagebox.showinfo("Wait", "Reading current GPS_DRV_OPTIONS, try again in a moment.")
            self.debug_var.set(0)
            return
        nid = self.selected_node
        if self.debug_var.get():
            new_val = self.last_options_seen | GPS_DRV_BIT_UBX_DEBUG
        else:
            new_val = self.last_options_seen & ~GPS_DRV_BIT_UBX_DEBUG
        self.worker.post(lambda: self.worker.request_set_gps_drv_options(nid, new_val))

    def _do_tunnel_start(self):
        if not self._require_selection():
            return
        if self.worker.tunnel is not None:
            messagebox.showwarning("Tunnel running", "Stop the current tunnel first.")
            return
        try:
            port = int(self.tunnel_port_var.get())
            host = self.tunnel_host_var.get().strip() or "127.0.0.1"
            serial_id = int(self.tunnel_serial_var.get())
            baud = int(self.tunnel_baud_var.get())
        except ValueError as ex:
            messagebox.showerror("Invalid input", str(ex))
            return
        lock = bool(self.tunnel_lock_var.get())
        target_node = self.selected_node

        ts = TunnelSession(
            worker=self.worker,
            target_node=target_node,
            serial_id=serial_id,
            baudrate=baud,
            lock_port=lock,
            listen_host=host,
            listen_port=port,
            log_cb=lambda line: self.log_line(line),
            status_cb=lambda s: self.root.after(0, self.tunnel_status_var.set, s),
        )
        try:
            ts.start()
        except OSError as ex:
            messagebox.showerror("Could not start tunnel", str(ex))
            return
        self.worker.tunnel = ts
        self.tunnel_start_btn.configure(state=tk.DISABLED)
        self.tunnel_stop_btn.configure(state=tk.NORMAL)

    def _do_tunnel_stop(self):
        ts = self.worker.tunnel
        if ts is None:
            return
        # Detach first so the worker stops feeding rx into a closing session.
        self.worker.tunnel = None
        ts.stop()
        self.tunnel_start_btn.configure(state=tk.NORMAL)
        self.tunnel_stop_btn.configure(state=tk.DISABLED)
        self.tunnel_bytes_var.set("")

    def _do_reboot(self):
        if not self._require_selection():
            return
        nid = self.selected_node
        if not messagebox.askyesno("Reboot", f"Reboot node {nid}? GPS will drop out briefly."):
            return
        self.worker.post(lambda: self.worker.request_restart(nid))

    # ---------------------- periodic refresh ----------------------
    def _refresh_gui(self):
        try:
            with self.worker._lock:
                nodes_snapshot = {nid: dict(rec) for nid, rec in self.worker.nodes.items()}
                gps_snapshot = dict(self.worker.gps_state.get(self.selected_node, {})) \
                    if self.selected_node is not None else {}
                opts = self.worker.gps_drv_options.get(self.selected_node) \
                    if self.selected_node is not None else None

            self.status_var.set(f"nodes: {len(nodes_snapshot)}")
            self._refresh_nodes_table(nodes_snapshot)
            self._refresh_gps_panel(gps_snapshot)

            if (self._await_fix_for_read
                    and self.selected_node is not None
                    and "last_fix2" in gps_snapshot):
                self._await_fix_for_read = False
                nid = self.selected_node
                self.opts_value_var.set("(reading…)")
                self.worker.post(lambda: self.worker.request_get_gps_drv_options(nid))

            ts = self.worker.tunnel
            if ts is not None:
                self.tunnel_bytes_var.set(f"TX {ts.bytes_tx} B / RX {ts.bytes_rx} B")

            if opts is not None and opts != self.last_options_seen:
                self.last_options_seen = opts
                self.opts_value_var.set(f"0x{opts:04x}  ({opts})")
                self._suppress_write = True
                try:
                    self.debug_var.set(1 if (opts & GPS_DRV_BIT_UBX_DEBUG) else 0)
                finally:
                    self._suppress_write = False
        finally:
            self.root.after(250, self._refresh_gui)

    def _refresh_nodes_table(self, nodes):
        existing = {self.tree.item(iid, "values")[0]: iid for iid in self.tree.get_children()}
        seen = set()
        for nid in sorted(nodes.keys()):
            rec = nodes[nid]
            row = (
                str(nid),
                rec.get("name", "?"),
                rec.get("hw_ver", "?"),
                rec.get("sw_ver", "?"),
                self._mode_str(rec.get("mode")),
                self._health_str(rec.get("health")),
                self._fmt_uptime(rec.get("uptime")),
                f"0x{rec.get('vendor', 0):04x}",
            )
            iid_key = str(nid)
            seen.add(iid_key)
            if iid_key in existing:
                self.tree.item(existing[iid_key], values=row)
            else:
                self.tree.insert("", tk.END, iid=iid_key, values=row)
        for k, iid in existing.items():
            if k not in seen:
                self.tree.delete(iid)

    def _refresh_gps_panel(self, s):
        if not s:
            for v in self.gps_vars.values():
                v.set("—")
            return
        fix_str = "{}/{}/{}".format(
            FIX2_STATUS.get(s.get("fix_status"), "?"),
            FIX2_MODE.get(s.get("fix_mode"), "?"),
            FIX2_SUBMODE.get(s.get("fix_submode"), "?"),
        )
        self.gps_vars["fix"].set(fix_str)
        self.gps_vars["sats"].set(f"{s.get('sats_used', '?')}/{s.get('sats_visible', '?')}")
        self.gps_vars["lat"].set(f"{s.get('lat_deg', 0.0):.7f}" if "lat_deg" in s else "—")
        self.gps_vars["lon"].set(f"{s.get('lon_deg', 0.0):.7f}" if "lon_deg" in s else "—")
        self.gps_vars["alt_msl"].set(f"{s.get('alt_msl_m', 0.0):.2f}" if "alt_msl_m" in s else "—")
        self.gps_vars["alt_ellip"].set(f"{s.get('alt_ellip_m', 0.0):.2f}" if "alt_ellip_m" in s else "—")
        self.gps_vars["vel_n"].set(f"{s.get('vel_n', 0.0):.2f}" if "vel_n" in s else "—")
        self.gps_vars["vel_e"].set(f"{s.get('vel_e', 0.0):.2f}" if "vel_e" in s else "—")
        self.gps_vars["vel_d"].set(f"{s.get('vel_d', 0.0):.2f}" if "vel_d" in s else "—")
        self.gps_vars["hdop"].set(f"{s.get('hdop', 0.0):.2f}" if "hdop" in s else "—")
        self.gps_vars["vdop"].set(f"{s.get('vdop', 0.0):.2f}" if "vdop" in s else "—")
        self.gps_vars["pdop"].set(f"{s.get('pdop', 0.0):.2f}" if "pdop" in s else "—")
        now = time.time()
        self.gps_vars["age_fix2"].set(f"{now - s['last_fix2']:.1f}" if "last_fix2" in s else "—")
        self.gps_vars["age_aux"].set(f"{now - s['last_aux']:.1f}" if "last_aux" in s else "—")

    @staticmethod
    def _fmt_uptime(secs):
        if secs is None:
            return "?"
        try:
            s = int(secs)
        except (TypeError, ValueError):
            return str(secs)
        h, rem = divmod(s, 3600)
        m, s = divmod(rem, 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    @staticmethod
    def _mode_str(mode):
        return {0: "OPERATIONAL", 1: "INITIALIZATION", 2: "MAINTENANCE",
                3: "SOFTWARE_UPDATE", 7: "OFFLINE"}.get(mode, str(mode))

    @staticmethod
    def _health_str(h):
        return {0: "OK", 1: "WARNING", 2: "ERROR", 3: "CRITICAL"}.get(h, str(h))
