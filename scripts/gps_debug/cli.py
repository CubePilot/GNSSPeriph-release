"""CLI entrypoint and welcome-window plumbing."""

import argparse
import os
import sys
import tkinter as tk
from multiprocessing import freeze_support

from .gui import GpsDebugGui
from .welcome import WelcomeDialog
from .worker import CanWorker


def parse_args():
    p = argparse.ArgumentParser(
        description="GPS debug GUI for CAN GPS units connected to a Cube autopilot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("port", nargs="?", default=None,
                   help="dronecan device URL (optional; if omitted welcome window asks for it)")
    p.add_argument("--node-id", type=int, default=127, help="local DroneCAN node ID (default 127)")
    p.add_argument("--bus", type=int, default=1, help="CAN bus number on the autopilot (default 1)")
    p.add_argument("--target-system", type=int, default=0, help="MAVLink target system id (0 = first seen)")
    p.add_argument("--baudrate", type=int, default=115200, help="serial baud (only for serial mavcan ports)")
    p.add_argument("--signing-key", default=os.environ.get("DRONECAN_SIGNING_KEY"),
                   help="MAVLink2 signing pass-phrase")
    p.add_argument("--no-welcome", action="store_true",
                   help="skip the welcome window (requires positional port)")
    return p.parse_args()


def _verify_pyserial():
    """Catch the 'pip install serial' vs 'pip install pyserial' mistake.

    The two packages share the `serial` import name. If the wrong one is
    installed, `import serial` succeeds but `serial.SerialException` /
    `serial.Serial` are missing, and the failure surfaces deep inside a
    multiprocessing child as `AttributeError: module 'serial' has no
    attribute 'SerialException'` — extremely confusing.
    """
    try:
        import serial as _s
    except ImportError:
        print("ERROR: pyserial is not installed.\n"
              "    pip install pyserial", file=sys.stderr)
        sys.exit(2)
    if not hasattr(_s, "SerialException") or not hasattr(_s, "Serial"):
        loc = getattr(_s, "__file__", "(unknown)")
        print(
            "ERROR: the wrong `serial` module is installed (probably the\n"
            "       unrelated 'serial' package, not 'pyserial'). Found at:\n"
            f"           {loc}\n"
            "       Fix:\n"
            "           pip uninstall -y serial\n"
            "           pip install --force-reinstall pyserial\n",
            file=sys.stderr,
        )
        sys.exit(2)


def main():
    freeze_support()
    _verify_pyserial()
    args = parse_args()

    defaults = dict(
        port=args.port,  # may be None — welcome dialog will fill from detected serial.
        node_id=args.node_id,
        bus=args.bus,
        target_system=args.target_system,
        baudrate=args.baudrate,
        signing_key=args.signing_key,
    )

    if args.no_welcome:
        if not args.port:
            print("--no-welcome requires a positional port URL", file=sys.stderr)
            sys.exit(2)
        cfg = defaults
    else:
        cfg = WelcomeDialog(defaults).run()
        if cfg is None:
            return  # user cancelled

    root = tk.Tk()
    worker = CanWorker(
        port=cfg["port"],
        node_id=cfg["node_id"],
        bus=cfg["bus"],
        target_system=cfg["target_system"],
        baudrate=cfg["baudrate"],
        signing_key=cfg["signing_key"],
        log_cb=lambda line: gui.log_line(line),
    )
    gui = GpsDebugGui(root, worker)
    worker.start()

    def on_close():
        if worker.tunnel is not None:
            try:
                worker.tunnel.stop()
            except Exception:
                pass
            worker.tunnel = None
        worker.stop()
        root.after(200, root.destroy)
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
