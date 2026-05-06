"""GPS debug GUI for CAN GPS units connected to a Cube autopilot.

Talks to the Cube over MAVLink, tunnels DroneCAN through it (CAN_FRAME),
discovers GPS nodes on the bus, displays node info / GPS fix telemetry,
toggles GPS_DRV_OPTIONS bits, and exposes a TCP <-> uavcan.tunnel.Targetted
serial bridge for u-blox debugging.

Run as:
    python -m gps_debug                     # welcome dialog then GUI
    python -m gps_debug mavcan:udpin:0.0.0.0:14550 --no-welcome
"""

from .cli import main

__all__ = ["main"]
