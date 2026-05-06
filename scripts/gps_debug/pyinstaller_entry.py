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
    # (1) — give multiprocessing somewhere safe to write tracebacks BEFORE
    # freeze_support runs the child target. Order matters: in a spawned child
    # under --windowed Windows, sys.stdout/sys.stderr are None and freeze_support
    # never returns (it dispatches to the target and exits), so any code below
    # freeze_support() does NOT execute in the child. If the child target
    # writes a traceback to stderr, the unhandled `NoneType.write` AttributeError
    # silently kills the io subprocess and the parent's queues go dry.
    if sys.stdout is None:
        sys.stdout = io.StringIO()
    if sys.stderr is None:
        sys.stderr = io.StringIO()

    # (2) — now safe to dispatch to the spawned child target if we're one.
    freeze_support()

    # (3) — chdir to the bundle directory before pymavlink imports anything.
    # `mavgen_python_dialect` calls `os.path.relpath(xml)` which compares the
    # bundled XML path (always on C: in --onedir, or in _MEIPASS for --onefile)
    # against the current working directory. When the user launches the .exe
    # from a Parallels / UTM \\Mac\Home network share, the cwd is on a UNC
    # mount and `relpath` raises `ValueError: path is on mount 'C:', start on
    # mount '\\Mac\Home'`. Switching to a directory on C: avoids it entirely.
    bundle_dir = getattr(sys, "_MEIPASS", None) or os.path.dirname(sys.executable)
    try:
        os.chdir(bundle_dir)
    except OSError:
        pass

    # (4) — defensive: silence pymavlink's runtime dialect regen if the
    # chdir above didn't help (e.g. unusual frozen layout). The bundle
    # already ships the generated dialect modules.
    os.environ.setdefault("MAVLINK20", "1")
    try:
        from pymavlink.generator import mavgen as _mavgen
        _mavgen.mavgen_python_dialect = lambda *a, **kw: True
    except Exception:
        pass

from gps_debug.cli import main

if __name__ == "__main__":
    main()
