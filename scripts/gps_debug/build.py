#!/usr/bin/env python3
"""Build a standalone single-file binary using PyInstaller.

Cross-platform: produces `dist/gps_debug_gui.exe` on Windows,
`dist/gps_debug_gui` on macOS / Linux.

Usage:
    pip install pyinstaller
    python -m gps_debug.build           # from inside scripts/
    # or
    python scripts/gps_debug/build.py   # from repo root

Output:
    dist/gps_debug_gui[.exe]            # the standalone binary
    build/                              # intermediate artefacts (safe to delete)

Notes:
- Bundles dronecan DSDL specs + pymavlink message XMLs (--collect-data).
- Uses --windowed so Windows users do not get a console window. Pass
  --console to keep the console attached (useful for debugging).
"""

import argparse
import importlib
import platform
import shutil
import subprocess
import sys
from pathlib import Path


REQUIRED_PACKAGES = ("dronecan", "pymavlink", "serial")


def _check_dependencies():
    """Make sure runtime deps are importable in the *build* environment, and
    return the parent directory of each so we can hand them to PyInstaller via
    --paths.

    PEP 660 editable installs use a custom MetaPathFinder rather than putting
    the package on sys.path, so PyInstaller's static analyzer cannot find them
    without an explicit --paths hint.
    """
    missing = []
    extra_paths = []
    for name in REQUIRED_PACKAGES:
        try:
            mod = importlib.import_module(name)
        except ImportError:
            missing.append(name)
            continue
        # __file__ points at the package's __init__.py — its grandparent is
        # the directory we'd add to sys.path to make the package importable.
        try:
            pkg_init = Path(mod.__file__).resolve()
            parent = pkg_init.parent.parent          # site-packages / source root
            if parent not in extra_paths:
                extra_paths.append(parent)
        except Exception:
            pass
    if missing:
        py = sys.executable
        print(
            "\nERROR: the following packages are not importable by the Python "
            "PyInstaller will run with:\n"
            f"    python = {py}\n"
            f"    missing = {', '.join(missing)}\n\n"
            "Install them in this same interpreter, e.g.:\n"
            f"    {py} -m pip install {' '.join(missing).replace('serial', 'pyserial')}\n"
            "If you use a local checkout of dronecan, install it editable:\n"
            f"    {py} -m pip install -e /path/to/pydronecan\n",
            file=sys.stderr,
        )
        sys.exit(2)
    return extra_paths


def _find_dronecan_dsdl():
    """Locate the directory containing dronecan's DSDL `.uavcan` files.

    A pip-installed dronecan ships them at `<pkg>/dsdl_specs/`. The local
    godronecan checkout used by the repo has them outside the package at
    `<godronecan>/DSDL/`. We probe both layouts and return the one that exists,
    or None if neither does.
    """
    import dronecan
    pkg_dir = Path(dronecan.__file__).resolve().parent
    candidates = [
        pkg_dir / "dsdl_specs",                          # pip layout
        pkg_dir.parent.parent / "DSDL",                  # godronecan layout
    ]
    for c in candidates:
        if (c / "uavcan").is_dir():
            return c
    return None


def main():
    here = Path(__file__).resolve().parent          # scripts/gps_debug/
    repo_root = here.parent.parent                   # repo root
    # PyInstaller treats the entry as a top-level script (no package context),
    # so we use a dedicated entrypoint that does absolute imports rather than
    # the package's __main__.py (which uses relative imports for `python -m`).
    entry = here / "pyinstaller_entry.py"

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", default="gps_debug_gui",
                        help="output binary name (default: gps_debug_gui)")
    parser.add_argument("--console", action="store_true",
                        help="keep console window (useful for debugging)")
    parser.add_argument("--onefile", action="store_true",
                        help="force a single-file build (default: --onedir on macOS, "
                             "--onefile elsewhere because macOS .app bundles cannot be one file)")
    parser.add_argument("--onedir", action="store_true",
                        help="force a folder build")
    parser.add_argument("--clean", action="store_true",
                        help="wipe build/ and dist/ before building")
    parser.add_argument("--icon", default=None,
                        help="path to a .ico (Windows) / .icns (macOS) icon file")
    parser.add_argument("--workdir", default=str(repo_root),
                        help="working directory for PyInstaller (default: repo root)")
    args = parser.parse_args()

    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print(f"PyInstaller is not installed. Run:  {sys.executable} -m pip install pyinstaller",
              file=sys.stderr)
        sys.exit(1)

    extra_paths = _check_dependencies()
    dsdl_src = _find_dronecan_dsdl()
    if dsdl_src is None:
        print(
            "\nERROR: could not locate dronecan's DSDL specs.\n"
            "Tried <pkg>/dsdl_specs and <pkg>/../../DSDL — neither exists.\n"
            "If you use a local checkout of godronecan, the DSDL should be at "
            "<godronecan>/DSDL/ (containing uavcan/, dronecan/, etc).\n",
            file=sys.stderr,
        )
        sys.exit(2)
    print(f"bundling DSDL from: {dsdl_src}")

    # Decide one-file vs one-dir.
    is_macos = platform.system() == "Darwin"
    if args.onedir and args.onefile:
        print("ERROR: --onedir and --onefile are mutually exclusive", file=sys.stderr)
        sys.exit(2)
    if args.onefile:
        mode_flag = "--onefile"
    else:
        # Default everywhere: --onedir.
        # macOS: PyInstaller deprecated --onefile + --windowed (cannot make a
        # single-file .app bundle).
        # Windows: --onefile extracts to a temp dir on launch, which on a UNC
        # shared filesystem causes pymavlink to crash on os.path.relpath
        # (cross-drive). --onedir keeps every file on the launcher's drive.
        # Linux: kept consistent with the others for predictable output shape.
        mode_flag = "--onedir"

    workdir = Path(args.workdir).resolve()
    if args.clean:
        for d in ("build", "dist"):
            p = workdir / d
            if p.exists():
                print(f"removing {p}")
                shutil.rmtree(p, ignore_errors=True)

    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--name", args.name,
        mode_flag,
        "--noconfirm",
        # Bundle the DSDL / message-definition data files these libs ship with.
        "--collect-data", "dronecan",
        "--collect-data", "pymavlink",
        # Pure-Python deps PyInstaller's auto-discovery sometimes misses.
        "--collect-submodules", "dronecan",
        "--collect-submodules", "serial",
        # pymavlink resolves dialect modules dynamically (set_dialect →
        # importlib.import_module("pymavlink.dialects.v20.ardupilotmega"));
        # without --collect-submodules they aren't bundled and import fails.
        "--collect-submodules", "pymavlink",
        # Keep the gps_debug package importable from inside the bundle.
        "--paths", str(here.parent),
    ]
    # Add filesystem paths to packages that are installed editable (PEP 660),
    # so PyInstaller's static analyzer can scan them.
    for p in extra_paths:
        cmd += ["--paths", str(p)]

    # Bundle the DSDL specs at dronecan/dsdl_specs inside the bundle so the
    # library's normal `get_resource_path("dronecan", "dsdl_specs")` lookup
    # works at runtime.
    sep = ";" if platform.system() == "Windows" else ":"
    cmd += ["--add-data", f"{dsdl_src}{sep}dronecan/dsdl_specs"]
    if args.console:
        cmd.append("--console")
    else:
        cmd.append("--windowed")
    if args.icon:
        cmd += ["--icon", args.icon]
    cmd.append(str(entry))

    print("running:", " ".join(cmd))
    proc = subprocess.run(cmd, cwd=str(workdir))
    if proc.returncode != 0:
        sys.exit(proc.returncode)

    out_dir = workdir / "dist"
    print(f"\nBuild complete. Artefacts in {out_dir}")
    if out_dir.exists():
        for p in sorted(out_dir.iterdir()):
            if p.is_file():
                size_mb = p.stat().st_size / (1024 * 1024)
                print(f"  {p.name}  ({size_mb:.1f} MB)")
            else:
                print(f"  {p.name}/  (directory)")


if __name__ == "__main__":
    main()
