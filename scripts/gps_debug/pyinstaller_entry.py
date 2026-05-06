"""Standalone entry point for PyInstaller bundles.

PyInstaller invokes the entry script directly (no package context), so the
relative import in `__main__.py` (`from .cli import main`) raises
`ImportError: attempted relative import with no known parent package`.
This file uses an absolute import instead and is what `build.py` points at.
The regular `python -m gps_debug` path still uses `__main__.py`.

Also applies two frozen-build workarounds before importing anything that
pulls in pymavlink, so they take effect for the very first MAVLink connect:

1. multiprocessing.freeze_support() must run as early as possible, otherwise
   the spawned child re-runs the entire entry on Windows.
2. With --windowed on Windows, sys.stdout/sys.stderr can be None in child
   processes; multiprocessing crashes with AttributeError if anything tries
   to .write() to them. We replace None streams with throw-away buffers.
3. pymavlink's `set_dialect` calls `os.path.relpath(xml, py_dir)` to decide
   whether to regenerate dialect modules. On Windows that raises ValueError
   when xml lives on C:\ (PyInstaller's _MEI temp dir) and py_dir lives on
   a network mount like \\Mac\Home (Parallels / UTM shared folder). Stub
   the regen call out — the pre-generated dialect modules are already in
   the bundle, so it has nothing useful to do anyway.
"""

import io
import os
import sys
from multiprocessing import freeze_support

if getattr(sys, "frozen", False):
    # (1) — must be the first thing the spawned worker sees.
    freeze_support()

    # (2) — give multiprocessing somewhere safe to write tracebacks.
    if sys.stdout is None:
        sys.stdout = io.StringIO()
    if sys.stderr is None:
        sys.stderr = io.StringIO()

    # (3) — silence pymavlink's runtime dialect regen.
    os.environ.setdefault("MAVLINK20", "1")
    try:
        from pymavlink.generator import mavgen as _mavgen
        _mavgen.mavgen_python_dialect = lambda *a, **kw: True
    except Exception:
        # If pymavlink layout changes, fall through and let the real error
        # surface in the log later rather than masking it here.
        pass

from gps_debug.cli import main

if __name__ == "__main__":
    main()
