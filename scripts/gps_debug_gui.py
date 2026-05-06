#!/usr/bin/env python3
"""Legacy entrypoint — the implementation now lives in the gps_debug package.

Equivalent invocations:
    ./scripts/gps_debug_gui.py                            # welcome dialog then GUI
    ./scripts/gps_debug_gui.py mavcan:udpin:0.0.0.0:14550 --no-welcome
    python -m gps_debug                                   # same, run as a module
"""

import os
import sys

# Make `from gps_debug import main` resolvable when running this file directly
# from anywhere — the package lives next to this shim.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gps_debug import main  # noqa: E402

if __name__ == "__main__":
    main()
