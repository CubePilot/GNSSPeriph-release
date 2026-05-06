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
datas += collect_data_files("dronecan")     # bundles uavcan/* DSDL specs
datas += collect_data_files("pymavlink")    # bundles MAVLink XML message defs

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
