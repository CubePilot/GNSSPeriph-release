"""Standalone entry point for PyInstaller bundles.

PyInstaller invokes the entry script directly (no package context), so the
relative import in `__main__.py` (`from .cli import main`) raises
`ImportError: attempted relative import with no known parent package`.
This file uses an absolute import instead and is what `build.py` points at.
The regular `python -m gps_debug` path still uses `__main__.py`.
"""

from gps_debug.cli import main

if __name__ == "__main__":
    main()
