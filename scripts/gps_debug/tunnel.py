"""TCP <-> uavcan.tunnel.Targetted bridge for u-blox debugging.

Pattern is borrowed from pydronecan's DroneCANSerial + tools/dronecan_tcplink.py:
broadcast Targetted with protocol=GPS_GENERIC, target_node=GPS_NODE; receive
Targetted broadcasts where source==target_node and message.target_node is us.
"""

import queue
import socket
import threading
import time

from .constants import TUNNEL_KEEPALIVE_S, TUNNEL_MAX_CHUNK


class TunnelSession:
    """Bidirectional bridge: TCP socket on the host <-> a serial port behind a
    DroneCAN GPS node. Lets u-center / pyubx2 / etc. talk to the u-blox chip
    exactly as if it were on a serial port (use socat or a TCP-aware tool to
    expose it as /dev/ttyXX if needed).
    """

    def __init__(self, worker, target_node, serial_id, baudrate, lock_port,
                 listen_host, listen_port, log_cb, status_cb):
        self.worker = worker
        self.target_node = target_node
        self.serial_id = serial_id
        self.baudrate = baudrate
        self.lock_port = lock_port
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.log_cb = log_cb
        self.status_cb = status_cb
        self._stop = threading.Event()
        self._sock = None
        self._client = None
        self._client_addr = None
        self._rx_q = queue.Queue()
        self._last_send = 0.0
        self.bytes_tx = 0
        self.bytes_rx = 0
        self._thread = threading.Thread(target=self._run, name="tunnel_io", daemon=True)

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.listen_host, self.listen_port))
        self._sock.listen(1)
        self._sock.settimeout(0.2)
        self._thread.start()
        lock_str = " (port locked)" if self.lock_port else ""
        self.log_cb(f"tunnel: listening on {self.listen_host}:{self.listen_port} -> "
                    f"node {self.target_node}, serial_id={self.serial_id}, "
                    f"baud={self.baudrate}{lock_str}")
        self.status_cb(f"listening on :{self.listen_port}")

    def stop(self):
        self._stop.set()
        try:
            if self._client is not None:
                try:
                    self._client.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self._client.close()
        finally:
            self._client = None
        try:
            if self._sock is not None:
                self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._thread.join(timeout=1.0)
        self.log_cb("tunnel: stopped")
        self.status_cb("stopped")

    def feed_rx(self, data):
        """Called from the CanWorker thread when a Targetted broadcast arrives."""
        if data:
            self.bytes_rx += len(data)
            try:
                self._rx_q.put_nowait(data)
            except queue.Full:
                pass

    # ----- internal -----
    def _enqueue_send(self, data):
        self._last_send = time.time()
        self.worker.post(lambda d=data: self.worker.tunnel_send_bytes(d))

    def _maybe_keepalive(self):
        if time.time() - self._last_send > TUNNEL_KEEPALIVE_S:
            self._enqueue_send(b"")

    def _run(self):
        while not self._stop.is_set():
            try:
                client, addr = self._sock.accept()
            except socket.timeout:
                # No client; keep the link warm so the GPS node accepts our
                # Targetted broadcasts immediately when one arrives.
                self._maybe_keepalive()
                self._drain_rx()
                continue
            except OSError:
                return  # socket closed by stop()
            client.settimeout(0.0)
            self._client = client
            self._client_addr = addr
            self.log_cb(f"tunnel: client connected from {addr[0]}:{addr[1]}")
            self.status_cb(f"connected from {addr[0]}:{addr[1]}")
            try:
                self._client_loop(client)
            finally:
                try:
                    client.close()
                except OSError:
                    pass
                self._client = None
                self._client_addr = None
                self.log_cb("tunnel: client disconnected")
                self.status_cb(f"listening on :{self.listen_port}")

    def _client_loop(self, sock):
        while not self._stop.is_set():
            # 1. host -> CAN
            try:
                data = sock.recv(4096)
                if data == b"":
                    return  # peer closed
                for i in range(0, len(data), TUNNEL_MAX_CHUNK):
                    chunk = data[i:i + TUNNEL_MAX_CHUNK]
                    self._enqueue_send(chunk)
                    self.bytes_tx += len(chunk)
            except (BlockingIOError, socket.timeout):
                pass
            except (ConnectionResetError, OSError):
                return

            # 2. CAN -> host
            try:
                while True:
                    pkt = self._rx_q.get_nowait()
                    sock.sendall(pkt)
            except queue.Empty:
                pass
            except (BrokenPipeError, OSError):
                return

            self._maybe_keepalive()
            time.sleep(0.005)

    def _drain_rx(self):
        try:
            while True:
                self._rx_q.get_nowait()
        except queue.Empty:
            pass
