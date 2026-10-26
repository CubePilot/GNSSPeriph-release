# HereProX logger field tools

Requires Python 3 and `pymavlink` (`pip install pymavlink`).

- `ubxr_to_ubx.py LOG.BIN -o drive.ubx` rebuilds the raw u-blox stream (RAWX, SFRBX, PVT, MON-RF, TIM-*, …) from the `UBXR` records. It prints the frame counts and the `Seq` gaps (dropped records). Then convert with RTKLIB: `convbin -r ubx drive.ubx` → RINEX OBS/NAV for PPK.
- `timesync_fit.py LOG.BIN` fits the MCU clock (`TimeUS`) to GPS time from `EXTS` + TIM-TM2. It prints offset, drift and rms, plus the `PPSE` + TIM-TP cross-check.
- Self-check: `python3 -m unittest discover -s scripts/logger -v`

Design and field procedure: `docs/superpowers/specs/2026-10-26-herepro-logger-design.md`.
