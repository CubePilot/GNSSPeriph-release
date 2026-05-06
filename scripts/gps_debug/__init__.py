"""GPS debug GUI for CAN GPS units connected to a Cube autopilot.

Talks to the Cube over MAVLink, tunnels DroneCAN through it (CAN_FRAME),
discovers GPS nodes on the bus, displays node info / GPS fix telemetry,
toggles GPS_DRV_OPTIONS bits, and exposes a TCP <-> uavcan.tunnel.Targetted
serial bridge for u-blox debugging.

Run from source:
    python -m gps_debug                     # welcome dialog then GUI
    python -m gps_debug mavcan:udpin:0.0.0.0:14550 --no-welcome

Build a standalone single-file binary (Windows .exe / macOS / Linux):
    pip install pyinstaller dronecan pymavlink pyserial
    python -m gps_debug.build               # cross-platform helper
    # or, equivalently:
    pyinstaller scripts/gps_debug/gps_debug_gui.spec
The output lands in dist/ — gps_debug_gui.exe on Windows. CI also produces
the binary on every push (see .github/workflows/build-gps-debug-gui.yml).
"""

from .cli import main

__all__ = ["main"]
