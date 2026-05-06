"""Background DroneCAN worker.

Owns the dronecan node + spin loop on a background thread. The GUI thread
talks to it through `post()` (an action queue) and reads shared state via
`self._lock`.

We deliberately do NOT use dronecan.app.node_monitor.NodeMonitor — it crashes
on malformed/partial GetNodeInfo replies (AttributeError: status). Instead we
maintain our own lightweight tracker via direct handlers + on-demand
GetNodeInfo requests.
"""

import queue
import threading
import time

import dronecan
from dronecan import uavcan

from .constants import (
    GPS_PARAM_NAME,
    RESTART_MAGIC,
    TUNNEL_MAX_CHUNK,
    TUNNEL_PROTOCOL_GPS_GENERIC,
)


class CanWorker(threading.Thread):
    _LOG_LEVELS = {0: "DEBUG", 1: "INFO", 2: "WARN", 3: "ERROR"}

    def __init__(self, port, node_id, bus, target_system, baudrate, signing_key, log_cb):
        super().__init__(daemon=True)
        self.port = port
        self.node_id = node_id
        self.bus = bus
        self.target_system = target_system
        self.baudrate = baudrate
        self.signing_key = signing_key
        self.log_cb = log_cb

        self.node = None
        self._info_pending = set()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.tunnel = None  # set by GUI thread when forwarding is active

        # GUI-readable shared state
        self.nodes = {}              # node_id -> {name, hw_ver, sw_ver, mode, health, uptime, vendor, last_seen}
        self.gps_state = {}          # node_id -> last Fix2/Auxiliary fields
        self.gps_drv_options = {}    # node_id -> latest known GPS_DRV_OPTIONS value
        self.gps_glitch = {}         # node_id -> {"reason": str, "ts": float}; latched until cleared
        self._prev_fix2 = {}         # node_id -> last Fix2 snapshot for delta-based detection

        self._action_q = queue.Queue()

    # ---------------------- thread control ----------------------
    def stop(self):
        self._stop.set()

    def post(self, fn):
        """Schedule `fn` to run on the worker thread (so dronecan calls stay single-threaded)."""
        self._action_q.put(fn)

    def run(self):
        try:
            kwargs = dict(
                node_id=self.node_id,
                bitrate=1000000,
                bus_number=self.bus,
                mavlink_target_system=self.target_system,
                baudrate=self.baudrate,
            )
            self.node = dronecan.make_node(self.port, **kwargs)
            if self.signing_key:
                self.node.can_driver.set_signing_passphrase(self.signing_key)

            self.node.add_handler(uavcan.equipment.gnss.Fix2, self._on_fix2)
            self.node.add_handler(uavcan.equipment.gnss.Auxiliary, self._on_aux)
            self.node.add_handler(uavcan.protocol.NodeStatus, self._on_status)
            self.node.add_handler(uavcan.protocol.debug.LogMessage, self._on_log_message)
            self.node.add_handler(uavcan.tunnel.Targetted, self._on_targetted)

            self.log_cb(f"connected: port={self.port} bus={self.bus} target_sys={self.target_system}")
        except Exception as ex:
            self.log_cb(f"connect failed: {ex}")
            return

        while not self._stop.is_set():
            try:
                while True:
                    fn = self._action_q.get_nowait()
                    try:
                        fn()
                    except Exception as ex:
                        self.log_cb(f"action error: {ex}")
            except queue.Empty:
                pass

            try:
                self.node.spin(timeout=0.1)
            except Exception as ex:
                self.log_cb(f"spin error: {ex}")

        try:
            self.node.close()
        except Exception:
            pass

    # ---------------------- handlers ----------------------
    def _on_status(self, event):
        nid = event.transfer.source_node_id
        m = event.message
        with self._lock:
            rec = self.nodes.setdefault(nid, {})
            prev_uptime = rec.get("uptime")
            rec["mode"] = m.mode
            rec["health"] = m.health
            rec["uptime"] = m.uptime_sec
            rec["vendor"] = m.vendor_specific_status_code
            rec["last_seen"] = time.time()
            need_info = "name" not in rec
            if prev_uptime is not None and m.uptime_sec < prev_uptime:
                # Node rebooted — refetch info.
                rec.pop("name", None)
                rec.pop("sw_ver", None)
                rec.pop("hw_ver", None)
                need_info = True
        if need_info and nid not in self._info_pending:
            self._info_pending.add(nid)
            try:
                self.node.request(uavcan.protocol.GetNodeInfo.Request(), nid,
                                  lambda ev, _nid=nid: self._on_info_response(_nid, ev))
            except Exception as ex:
                self._info_pending.discard(nid)
                self.log_cb(f"GetNodeInfo({nid}) request error: {ex}")

    def _on_info_response(self, nid, event):
        self._info_pending.discard(nid)
        if event is None:
            return  # timeout — _on_status will retry next time we see it
        try:
            r = event.response
            with self._lock:
                rec = self.nodes.setdefault(nid, {})
                try:
                    rec["name"] = bytes(r.name).decode("utf-8", errors="replace")
                except Exception:
                    pass
                try:
                    sv = r.software_version
                    rec["sw_ver"] = (f"{sv.major}.{sv.minor} "
                                     f"(vcs={sv.vcs_commit:08x} crc={sv.image_crc:016x})")
                except Exception:
                    pass
                try:
                    hv = r.hardware_version
                    rec["hw_ver"] = f"{hv.major}.{hv.minor}"
                    try:
                        rec["uid"] = bytes(hv.unique_id).hex()
                    except Exception:
                        pass
                except Exception:
                    pass
        except Exception as ex:
            self.log_cb(f"GetNodeInfo({nid}) parse error: {ex}")

    def _on_fix2(self, event):
        nid = event.transfer.source_node_id
        m = event.message
        now = time.time()
        with self._lock:
            s = self.gps_state.setdefault(nid, {})
            prev = self._prev_fix2.get(nid)
            s["fix_status"] = m.status
            s["fix_mode"] = m.mode
            s["fix_submode"] = m.sub_mode
            s["sats_used"] = m.sats_used
            s["lat_deg"] = m.latitude_deg_1e8 * 1e-8
            s["lon_deg"] = m.longitude_deg_1e8 * 1e-8
            s["alt_msl_m"] = m.height_msl_mm * 1e-3
            s["alt_ellip_m"] = m.height_ellipsoid_mm * 1e-3
            s["pdop"] = m.pdop
            s["vel_n"] = m.ned_velocity[0] if len(m.ned_velocity) > 0 else 0.0
            s["vel_e"] = m.ned_velocity[1] if len(m.ned_velocity) > 1 else 0.0
            s["vel_d"] = m.ned_velocity[2] if len(m.ned_velocity) > 2 else 0.0
            s["last_fix2"] = now
            # Snapshot for next-tick delta comparison.
            cur = {k: s[k] for k in
                   ("fix_status", "sats_used", "lat_deg", "lon_deg", "alt_msl_m")}
            cur["ts"] = now
            self._prev_fix2[nid] = cur

        if prev is not None:
            self._check_glitch(nid, prev, cur)

    def _check_glitch(self, nid, prev, cur):
        """Compare consecutive Fix2 samples; latch a glitch if any rule trips.

        Latched glitches stay set until clear_glitch(nid) is called from the GUI.
        Subsequent events do not overwrite an already-latched reason.
        """
        with self._lock:
            if nid in self.gps_glitch:
                return  # already latched, stay latched until reset
        reason = None
        # 1. Sat count drop — only trip if we *had* a healthy view before.
        if prev["sats_used"] >= 6 and cur["sats_used"] <= prev["sats_used"] - 4:
            reason = (f"sats dropped {prev['sats_used']} -> {cur['sats_used']}")
        # 2. Fix downgrade from 3D fix.
        elif prev["fix_status"] == 3 and cur["fix_status"] < 3:
            reason = f"fix downgraded {prev['fix_status']} -> {cur['fix_status']}"
        # 3. Position jump while staying 3D-fixed.
        elif prev["fix_status"] == 3 and cur["fix_status"] == 3:
            dist = self._haversine_m(prev["lat_deg"], prev["lon_deg"],
                                     cur["lat_deg"],  cur["lon_deg"])
            dt = max(cur["ts"] - prev["ts"], 0.05)
            if dist > 50.0 and dt < 2.0:
                reason = f"position jump {dist:.0f} m in {dt:.2f} s"
            elif abs(cur["alt_msl_m"] - prev["alt_msl_m"]) > 30.0 and dt < 2.0:
                reason = f"altitude jump {prev['alt_msl_m']:.1f} -> {cur['alt_msl_m']:.1f} m"
        if reason is not None:
            with self._lock:
                self.gps_glitch[nid] = {"reason": reason, "ts": cur["ts"]}
            self.log_cb(f"node {nid}: GPS GLITCH — {reason}")

    def clear_glitch(self, nid):
        """Re-arm detection for the given node."""
        with self._lock:
            self.gps_glitch.pop(nid, None)
            # Drop the prev sample so the next Fix2 establishes a fresh baseline
            # rather than tripping again on the same delta the user just saw.
            self._prev_fix2.pop(nid, None)
        self.log_cb(f"node {nid}: glitch state cleared, re-armed")

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        from math import radians, sin, cos, asin, sqrt
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
        return 2 * 6371000.0 * asin(sqrt(a))

    def _on_aux(self, event):
        nid = event.transfer.source_node_id
        m = event.message
        with self._lock:
            s = self.gps_state.setdefault(nid, {})
            s["hdop"] = m.hdop
            s["vdop"] = m.vdop
            s["sats_visible"] = m.sats_visible
            s["last_aux"] = time.time()

    def _on_log_message(self, event):
        m = event.message
        nid = event.transfer.source_node_id
        try:
            level = self._LOG_LEVELS.get(m.level.value, str(m.level.value))
            source = bytes(m.source).decode("utf-8", errors="replace")
            text = bytes(m.text).decode("utf-8", errors="replace")
        except Exception as ex:
            self.log_cb(f"node {nid}: log decode error: {ex}")
            return
        prefix = f"[{level}]"
        if source:
            prefix += f" {source}:"
        self.log_cb(f"node {nid} {prefix} {text}")

    def _on_targetted(self, event):
        """Forward incoming u-blox bytes to the active TunnelSession (if any)."""
        tm = self.tunnel
        if tm is None:
            return
        if event.transfer.source_node_id != tm.target_node:
            return
        if event.message.target_node != self.node_id:
            return
        try:
            tm.feed_rx(bytes(event.message.buffer))
        except Exception as ex:
            self.log_cb(f"tunnel rx error: {ex}")

    # ---------------------- actions (called via post()) ----------------------
    def request_get_gps_drv_options(self, target_node_id):
        req = uavcan.protocol.param.GetSet.Request()
        req.name = GPS_PARAM_NAME

        def cb(event):
            if event is None:
                self.log_cb(f"node {target_node_id}: GetSet timeout")
                return
            r = event.response
            if len(r.name) == 0:
                self.log_cb(f"node {target_node_id}: {GPS_PARAM_NAME} not present")
                return
            val = self._extract_int(r.value)
            if val is None:
                self.log_cb(f"node {target_node_id}: {GPS_PARAM_NAME} unexpected value type")
                return
            with self._lock:
                self.gps_drv_options[target_node_id] = val
            self.log_cb(f"node {target_node_id}: {GPS_PARAM_NAME} = 0x{val:04x} ({val})")

        self.node.request(req, target_node_id, cb)

    def request_set_gps_drv_options(self, target_node_id, new_value):
        req = uavcan.protocol.param.GetSet.Request()
        req.name = GPS_PARAM_NAME
        req.value.integer_value = int(new_value)

        def cb(event):
            if event is None:
                self.log_cb(f"node {target_node_id}: SET timeout")
                return
            r = event.response
            val = self._extract_int(r.value)
            if val is None:
                self.log_cb(f"node {target_node_id}: SET unexpected response value")
                return
            with self._lock:
                self.gps_drv_options[target_node_id] = val
            self.log_cb(f"node {target_node_id}: {GPS_PARAM_NAME} <- 0x{val:04x} (reboot to apply)")

        self.node.request(req, target_node_id, cb)

    def request_restart(self, target_node_id):
        req = uavcan.protocol.RestartNode.Request()
        req.magic_number = RESTART_MAGIC

        def cb(event):
            if event is None:
                self.log_cb(f"node {target_node_id}: RESTART (no reply, normal)")
                return
            self.log_cb(f"node {target_node_id}: RESTART ok={event.response.ok}")

        self.node.request(req, target_node_id, cb)

    def tunnel_send_bytes(self, data):
        """Broadcast one Targetted chunk (caller must keep it <= 120 bytes)."""
        tm = self.tunnel
        if tm is None or self.node is None:
            return
        msg = uavcan.tunnel.Targetted()
        msg.protocol.protocol = TUNNEL_PROTOCOL_GPS_GENERIC
        msg.target_node = tm.target_node
        msg.serial_id = tm.serial_id
        msg.options = msg.OPTION_LOCK_PORT if tm.lock_port else 0
        msg.baudrate = tm.baudrate
        msg.buffer = bytearray(data[:TUNNEL_MAX_CHUNK])
        try:
            self.node.broadcast(msg)
        except Exception as ex:
            self.log_cb(f"tunnel tx error: {ex}")

    @staticmethod
    def _extract_int(value):
        active = getattr(value, "union_field", None) or getattr(value, "_union_field", None)
        if active == "integer_value":
            return int(value.integer_value)
        if active == "real_value":
            return int(value.real_value)
        if active == "boolean_value":
            return int(bool(value.boolean_value))
        try:
            return int(value.integer_value)
        except Exception:
            return None
