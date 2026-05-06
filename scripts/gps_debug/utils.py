"""Shared GUI / OS helpers."""

import tkinter as tk
from tkinter import ttk

try:
    from serial.tools import list_ports as _serial_list_ports
except ImportError:  # pyserial ships with pymavlink, so this is unlikely
    _serial_list_ports = None


def use_hover_friendly_theme(root):
    """Pick a ttk theme whose Combobox dropdown highlights track the mouse.

    The macOS 'aqua' theme uses a native popdown that ignores Motion bindings,
    so dropdowns feel dead — no hover highlight. 'clam' (and 'alt'/'default')
    use a real Listbox popdown which does track hover.
    """
    style = ttk.Style(root)
    available = style.theme_names()
    for theme in ("clam", "alt", "default"):
        if theme in available:
            style.theme_use(theme)
            break
    root.bind_class("TCombobox", "<Map>", _hook_combobox_popdown, add="+")


def _hook_combobox_popdown(event):
    cbo = event.widget
    try:
        cbo.tk.eval(f"""
            set lb [ttk::combobox::PopdownWindow {cbo}].f.l
            bind $lb <Motion> {{
                %W selection clear 0 end
                %W selection set @%x,%y
                %W activate @%x,%y
            }}
        """)
    except tk.TclError:
        pass


def detect_serial_ports():
    """Return list of (device, description) for all serial ports the OS reports."""
    if _serial_list_ports is None:
        return []
    out = []
    for p in _serial_list_ports.comports():
        desc = p.description if p.description and p.description != "n/a" else ""
        out.append((p.device, desc))
    out.sort(key=lambda x: x[0])
    return out
