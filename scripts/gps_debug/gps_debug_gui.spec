# PyInstaller spec for gps_debug_gui — single-file standalone build.
# Build with:  pyinstaller scripts/gps_debug/gps_debug_gui.spec
# (Run from the repo root so relative paths resolve correctly.)

# -*- mode: python ; coding: utf-8 -*-
import sys
from pathlib import Path
from PyInstaller.utils.hooks import collect_data_files, collect_submodules

HERE = Path(SPECPATH)                   # scripts/gps_debug/
# Use the absolute-imports entrypoint — PyInstaller runs it as a top-level
# script, so __main__.py's relative imports would fail.
ENTRY = str(HERE / "pyinstaller_entry.py")

datas = []
datas += collect_data_files("dronecan")     # picks up dsdl_specs only when present
datas += collect_data_files("pymavlink")    # bundles MAVLink XML message defs

# Locate DSDL: pip layout has it at <pkg>/dsdl_specs/, the local godronecan
# checkout has it at <godronecan>/DSDL/. Bundle whichever exists at the path
# the runtime loader expects (dronecan/dsdl_specs).
import dronecan as _dc
_pkg = Path(_dc.__file__).resolve().parent
for _candidate in (_pkg / "dsdl_specs", _pkg.parent.parent / "DSDL"):
    if (_candidate / "uavcan").is_dir():
        datas.append((str(_candidate), "dronecan/dsdl_specs"))
        break
else:
    raise SystemExit("Cannot locate dronecan DSDL specs (no dsdl_specs/ or "
                     "godronecan-style ../DSDL/ found next to dronecan package)")

hiddenimports = []
hiddenimports += collect_submodules("dronecan")
hiddenimports += collect_submodules("serial")

block_cipher = None

a = Analysis(
    [ENTRY],
    pathex=[str(HERE.parent)],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="gps_debug_gui",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,                      # set True if upx is on PATH and you want compression
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,                  # Windows: no console; pass True if debugging
    disable_windowed_traceback=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,                      # set to "scripts/gps_debug/icon.ico" if available
)
