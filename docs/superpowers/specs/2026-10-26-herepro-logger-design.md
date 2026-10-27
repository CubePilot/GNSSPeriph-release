# HereProX Logger — Design

Date: 2026-10-26 · Status: draft for review (user answers folded in) · Scope: design only (no hwdef or source edits yet)

## Purpose

Record real drive logs (and later flight logs) for offline replay while we develop the tightly coupled GNSS/INS filter with GSF heading for Here4. The logging platform is a **HereProX**: HerePro hardware (ICM-42688, RM3100, eMMC, USB mass storage) with a u-blox **X20P** in place of the F9P. The first data set comes from a test team driving a car around a city with the unit standalone on battery: no flight controller and no DroneCAN bus.

Every `file:line` below refers to this repo at `b84ff44` (submodule `ardupilot` at `79b94643fb`).

What the test team sees:

```
Power bank → HereProX USB port. Purple for 5 s, then blue blinking, then green blinking once there's a fix.
... drive ...
Unplug the power bank. Plug HereProX USB into the laptop → purple LED, drive appears →
  APM/LOGS/00000007.BIN  (300 MB)
Copy it to  hereprox-1234_2026-11-02_city-loop_L07.bin
```

---

## Hardware Baseline

**HereProX is HerePro hardware with an X20P in place of the F9P, on the same UARTs and pins** (user confirmed). There is no separate HereProX hwdef; the logger variant includes `HerePro/hwdef.dat` directly.

| Feature | HerePro hwdef |
|---|---|
| MCU / flash | STM32H757, 2 MB, 256 KB BL reserve (`HerePro/hwdef.dat:5,26-30,89`) |
| IMU | ICM-42688 on SPI4 (`HerePro/hwdef.dat:138,152`) |
| Mag | RM3100 on SPI1, `ROTATION_PITCH_180` (`HerePro/hwdef.dat:150,156`) |
| eMMC | SDMMC1 8-bit, `HAL_SDMMC_TYPE_EMMC` (`HerePro/hwdef.dat:159-171,175`) |
| USB MSD | `HAL_USE_USB_MSD 1` on OTG2 HS (`HerePro/hwdef.dat:240`) |
| MCU→GNSS EXTINT | PD14 `GPIO(101)` = `GPS_EXTINT` (`HerePro/hwdef.dat:185,201`); user confirmed wired |
| GNSS timepulse→MCU | user confirmed wired. `CONFIGURE_PPS_PIN TRUE` (`:84,221`) only configures the receiver's TP5 output; no `HAL_GPIO_PPS` is defined, so no MCU interrupt is attached today (`AP_GPS_UBLOX.cpp:1377-1379`). **Which MCU pin is unconfirmed**: the only GNSS-side input with a GPIO number is PG2 `F97 INPUT GPIO(100)` (`:187`), so the design assumes `HAL_GPIO_PPS 100` and confirms it in the bench checklist. |

**Which UART the X20P sits on.** The driver's port is **USART3**:
- `SERIAL_ORDER OTG1 EMPTY USART3 UART7` makes USART3 = SERIAL2 (`HerePro/hwdef.dat:36,41-42`), and `SERIAL2_PROTOCOL 5` (GPS) is set in `HerePro/defaults.parm:7`.
- AP_GPS opens only GPS-protocol ports, with `GPS_MAX_RECEIVERS 1` (`:111`) and one receiver's control pins (`:181-188`).
- UART7 is opened at 115200 (`AP_Periph/AP_Periph.cpp:99`), but no AP_Periph code reads it. It is presumably the receiver's UART2, but this is **inferred, not verified**.

The logger uses only the receiver's UART1 via USART3. Every `*_UART1` key below assumes USART3 is connected to X20 UART1 (bench checklist item).

## Build Variant

Add a new board directory, `HereProX-logger/`. `wscript:33-60` auto-symlinks any top-level folder that contains `hwdef.dat`. The release HerePro firmware stays untouched.

`HereProX-logger/hwdef.dat`:

```
include ../HerePro/hwdef.dat

undef HAL_LOGGING_ENABLED AP_INERTIALSENSOR_ENABLED
define HAL_LOGGING_ENABLED 1
define AP_INERTIALSENSOR_ENABLED 1
define AP_PERIPH_LOGGER 1              # gates all logger-only code below
define UBLOX_MAX_RXM_RAWX_SATS 255     # needs #ifndef guard in AP_GPS_UBLOX.h:63
define HAL_LOGGING_FILE_BUFSIZE 400
define HAL_GPIO_PPS 100                # PG2 F97, assumed timepulse; confirm on bench
```

- `include` resolves relative to the including file (`chibios_hwdef.py:838-850`). `undef` also strips the earlier `define` lines (`chibios_hwdef.py:3046-3049`), so redefining is clean. ARK_RTK_GPS uses the same pattern (`ardupilot/libraries/AP_HAL_ChibiOS/hwdef/ARK_RTK_GPS/hwdef.dat:119-120`).
- `HereProX-logger/defaults.parm` starts with `@include ../HerePro/defaults.parm` (handled at `chibios_hwdef.py:2818-2832,2845`) and adds the parameters listed under Logging Start and Retrieval.
- Add `'HereProX-logger'` to the target list in `AP_Periph/wscript:24`, or the AP_Periph program is not built for that board. CI (`.github/workflows/c-cpp.yml`) is left alone for now.
- Keep `APJ_BOARD_ID 1037` and `CAN_APP_NODE_NAME` from HerePro (`HerePro/hwdef.dat:24,219`) so the existing bootloader accepts the image. Its version string (and the `HereProX-logger` board name in the `VER`/`MSG` startup lines) tells it apart from release firmware.

## Logger Wiring in AP_Periph

AP_Periph includes `AP_Logger.h` (`AP_Periph/AP_Periph.h:10`) but never creates a logger object. The parameter keys `k_param_logger` and `k_param_log_bitmask` are already reserved (`AP_Periph/Parameters.h:38-39`). Port the upstream wiring, all under `#if AP_PERIPH_LOGGER`:

| Change | Upstream reference |
|---|---|
| `AP_Logger logger;` and `static const LogStructure log_structure[]` members, `AP_Int32 log_bitmask` | `ardupilot/Tools/AP_Periph/AP_Periph.h:395-397`, `Parameters.h:187-188` |
| `GOBJECT(logger,"LOG",AP_Logger)` and `GSCALAR(log_bitmask,"LOG_BITMASK",…)` | `ardupilot/Tools/AP_Periph/Parameters.cpp:448-458` |
| `logger.init(g.log_bitmask, log_structure, …)` at the top of `init()` | `ardupilot/Tools/AP_Periph/AP_Periph.cpp:133-134` |
| `logger.periodic_tasks()` in `update()` | `ardupilot/Tools/AP_Periph/AP_Periph.cpp:511-512` |
| `gps.set_log_gps_bit()`, `compass.set_log_bit()`, `imu.set_log_raw_bit()` | default `-1` = never log (`AP_GPS.h:638`, `AP_Compass.h:526`, `AP_InertialSensor.h:243,675`) |

Two existing AP_Periph behaviours must change for a standalone unit:

1. **DNA gate.** GPS and mag only update after DroneCAN node allocation finishes (`AP_Periph/can.cpp:720-731`). A car with no CAN bus never allocates, so nothing gets logged. Fix: `CAN_NODE 125` in the logger defaults. A static ID is set at interface init (`can.cpp:562-563,647`), which marks DNA finished (`can.cpp:511-513`).
2. **IMU thread** (`AP_Periph/sensor_imu.cpp:10-39`):
   - It `return`s on a repeated timestamp (`:18-19`), which ends the thread permanently.
   - It broadcasts `RawIMU` on every loop (`:38`), which would flood a vehicle CAN bus at 400 Hz.
   - In the logger build: write the log, `continue` instead of `return`, and skip the broadcast.

## X20 Receiver Configuration

**The ArduPilot driver cannot configure an X20.**
- MON-VER detection only recognises the F9 (`00190000` + `EXT CORE 1`), the M9 and the M10 (`000A0000`) (`AP_GPS_UBLOX.cpp:1525-1560`). An X20 stays `UNKNOWN`, so `supports_F9_config()` is false (`:2533-2536`) and every VALSET path is skipped.
- The fallback path sets message rates with legacy `CFG-MSG` (`AP_GPS_UBLOX.cpp:2137`). It also polls `NAV-SVINFO` (`:367-371`) and `CFG-TP5` (`:393-395`).
- The X20 HPG 2.02 description lists only `CFG-CFG/RST/VALDEL/VALGET/VALSET` under CFG, with no legacy `CFG-MSG/PRT/RATE/NAV5/GNSS/TP5`, and no `NAV-SVINFO`, `MON-HW` or `MON-HW2` (`u-blox-X20-HPG-2.02.json`, `messages.UBX.CFG._list`, `MON._list`). The JSON is u-blox's *filtered* public file (`meta.configuration.filtered`), so absence is strong evidence but not proof.

**Decision: the logger configures the X20 itself, and the driver only parses.**
- Set `GPS_AUTO_CONFIG 0`. The driver then stops sending config (`AP_GPS_UBLOX.cpp:318-320`).
- AP_Periph sends one VALSET (**RAM layer only**; user OK, and release firmware is unaffected after a reflash or power cycle) through `AP_GPS::inject_data()` (`AP_GPS.h:760-761`) once the driver reports the port is detected. It repeats the VALSET until a `UBX-ACK-ACK` is seen in the raw stream.
- The baud change goes last. The driver re-detects at 921600 (`AP_GPS.cpp:74` includes 921600).
- The F9 boot-config prime still runs regardless of auto-config (`AP_GPS_UBLOX.cpp:313-315`). It uses legacy `CFG-MSG` and gives up after 30 tries (`:2148-2155`), so on an X20 it costs only startup time.
- Fixing the driver for X20 (MON-VER string, VALSET-only rates, CFG-TP keys) is product work and out of scope here; it is listed as a risk.

All keys below are verified in `u-blox-X20-HPG-2.02.json` (`configuration.<GROUP>.items.<NAME>.configKey`):

| Key | ID | Value | Why |
|---|---|---|---|
| CFG-RATE-MEAS | 0x30210001 | 200 | 5 Hz measurements |
| CFG-RATE-NAV | 0x30210002 | 1 | nav solution on every epoch |
| CFG-NAVSPG-DYNMODEL | 0x20110021 | 4 (AUTOMOT) | car drive; 6/7 (AIR1/AIR2) for flight |
| CFG-UART1OUTPROT-NMEA | 0x10740002 | 0 | free the bandwidth |
| CFG-MSGOUT-UBX_NAV_PVT_UART1 | 0x20910007 | 1 | the driver's fix source |
| CFG-MSGOUT-UBX_NAV_DOP_UART1 | 0x20910039 | 1 | the driver uses DOP |
| CFG-MSGOUT-UBX_RXM_RAWX_UART1 | 0x209102a5 | 1 | raw observables (with `sigId`, offset 22 of each 32-byte block) |
| CFG-MSGOUT-UBX_RXM_SFRBX_UART1 | 0x20910232 | 1 | broadcast ephemeris (up to 16 words per frame) |
| CFG-MSGOUT-UBX_MON_RF_UART1 | 0x2091035a | 5 | per-band AGC / noise / CW jamming; replaces MON-HW, which the X20 doesn't have |
| CFG-MSGOUT-UBX_NAV_SIG_UART1 | 0x20910346 | 5 | per-signal used/quality flags for blockage analysis |
| CFG-MSGOUT-UBX_NAV_SAT_UART1 | 0x20910016 | 5 | elevation/azimuth/health |
| CFG-MSGOUT-UBX_MON_SYS_UART1 | 0x2091069e | 5 | receiver CPU load and temperature (log health) |
| CFG-MSGOUT-UBX_TIM_TM2_UART1 | 0x20910179 | 1 | EXTINT time marks (Time Alignment) |
| CFG-MSGOUT-UBX_TIM_TP_UART1 | 0x2091017e | 1 | GNSS time of each timepulse (PPS cross-check) |
| CFG-UART1-BAUDRATE | 0x40520001 | 921600 | sent last |

**Signals are not touched** (user decision). There is no `CFG-SIGNAL-*` or `CFG-SIGNAL-PLAN` write: the unit keeps the plan it ships with. The logger records what is tracked (MON-GNSS, NAV-SIG, VALGET readback, see Self-Describing Logs). No QZSS L6/CLAS.

**Timepulse:** not reconfigured either. The public X20 keys only document TP2 (`CFG-TP-*_TP2`), so the logger reads back `CFG-TP-*` via VALGET and uses whatever pulse the unit outputs (expected 1 Hz).

## Raw UBX Capture

Decision on gap (a): **store complete raw UBX frames inside the `.bin`** as a new message, `UBXR`. Don't write a separate `.ubx` file and don't extend `GRXS`.

The gap: `GRXS` (`AP_GPS/LogStructure.h:189-204`) is built from a struct that names `sigId` as `reserved2` (`AP_GPS_UBLOX.h:593`). L1/L2/L5 measurements of the same satellite can't be told apart in it.

| Option | Verdict |
|---|---|
| Extend `GRXS` with `sigId` | Fixes RAWX only. SFRBX, MON-RF, NAV-SIG/SAT and MON-SYS would each need a new struct and decoder. Doubles RAWX bandwidth if we keep both. |
| Separate `.ubx` file (as rover mode does, `AP_Periph/GPS_Rover.cpp:66-96`) | Two concurrent FATFS writers, and the rover path `fsync`s every frame (`:95-96`). No MCU timestamp per frame, so it can't be aligned to the IMU. Two files can be separated in the field. |
| **`UBXR` in `.bin`** | Lossless for every message class with one generic hook. Each chunk carries MCU `TimeUS`, so arrival time sits on the same clock as the IMU. One file per drive. A short script rebuilds a `.ubx` for RTKLIB `convbin`. |

`UBXR` format: `TimeUS:Q, Seq:H, Len:B, Data0:a, Data1:a` (128 bytes payload, 143 bytes per record).
- `'a'` is a raw `int16[32]` memcpy (`AP_Logger_Backend.cpp:329-334`). `'Z'` would truncate at the first NUL (`:337-342`), so it can't be used.
- `Seq` increments per record and detects drops.
- A frame spans `ceil(len/128)` records. The extractor concatenates records in `Seq` order and re-validates the UBX checksum.

Driver changes in the `ardupilot` submodule (same files as the existing boot-prime commit):

1. Add one hook just before `_parse_gps()` (`AP_GPS_UBLOX.cpp:841`) that hands every checksum-valid frame to `_gps_raw_cb`. The existing RAWX-only copy (`:1578-1598`) is then redundant in the logger build.
2. Raise the RAWX limit. `UBLOX_MAX_RXM_RAWX_SATS` is 80 (`AP_GPS_UBLOX.h:63`), and frames bigger than `_buffer` are thrown away as noise before the checksum (`AP_GPS_UBLOX.cpp:793-799`). That includes the passthrough. All-band X20 epochs (about 90-150 measurements in open sky) exceed 80, so add an `#ifndef` guard and set 255 in the hwdef (buffer ≈ 8.2 KB, one instance).
3. Keep `unexpected_message()` (`AP_GPS_UBLOX.cpp:1050-1061`) from auto-disabling SFRBX/MON-RF/NAV-SIG. On the X20 it sends legacy `CFG-MSG`, which per the description isn't supported, but gate it on `GPS_AUTO_CONFIG` anyway so an F9 unit behaves the same.

`_gps_raw_cb` is a single slot (`AP_GPS.h:599`) also used by rover mode (`GPS_Rover.cpp:43`). The logger build keeps `ROVER_ENABLE 0`.

`GPS_RAW_DATA 0`: decoded `GRXH/GRXS` would duplicate about 27 kB/s and lack `sigId`. The decoded `GPS`/`GPA` messages (`AP_GPS.cpp:1925-1962`) stay on for quick-look plots in MAVExplorer.

## IMU Capture

Both rates are logged, for gap (c):

| Message | Content | Rate | Source |
|---|---|---|---|
| `ACC` / `GYR` (existing) | pre-filter sensor samples with per-sample `SampleUS` | **1 kHz for the car campaign** (user decision): ICM-42688 base ODR (`AP_InertialSensor_Invensensev3.cpp:694`) with `INS_GYRO_RATE 0`. The H7 default would be 2 kHz (`AP_InertialSensor.cpp:84-85`). | `log_accel_raw`/`log_gyro_raw` (`AP_InertialSensor_Backend.cpp:443-470,706-722`), enabled by `imu.set_log_raw_bit()` (filter state is `Pre`, `INS_RAW_LOG_OPT 0`) |
| `IMUD` (new) | `TimeUS, DelAngX/Y/Z, DelAngDT, DelVelX/Y/Z, DelVelDT, Temp` | loop rate 400 Hz (`IMU_SAMPLE_RATE 400`; range 0-1000, `AP_Periph/Parameters.cpp:213`) | `imu.get_delta_angle/velocity` (as already used in `sensor_imu.cpp:23-31`): the coning-corrected integrals the filter consumes |

- `INS_GYR_CAL 0`. Otherwise boot-time gyro calibration runs (`AP_InertialSensor.cpp:934-937`) and silently bakes a bias into the data whenever the car is moving at power-on. The filter must estimate bias anyway.
- Accel offsets stay at their default of zero, and `INS_*` params are recorded in `PARM`.
- `INS_GYRO_RATE 1` (2 kHz) is available for later flights; it doubles the raw IMU log bandwidth.

## Time Alignment

Every `.bin` `TimeUS` is MCU `micros64` (from the 24 MHz crystal on HerePro, `HerePro/hwdef.dat:19`). The GNSS epoch lives in RAWX `rcvTow`. MCU arrival time of a 4 KB RAWX frame lags the epoch by tens of ms, so it is not usable as the measurement time.

The primary method is an **EXTINT strobe with TIM-TM2** (user decision). Once a second, AP_Periph:
- takes `t = micros64()` and raises `GPS_EXTINT` inside a `chSysLock` section,
- writes `EXTS: TimeUS(edge), Count`,
- lowers the pin 10 ms later.

The X20 timestamps the edge in GNSS time (TIM-TM2 `count, wnR, towMsR, towSubMsR, accEst`; field layout matches `ubx_tim_tm2`, `AP_GPS_UBLOX.h:622-633`), and the frame lands in `UBXR`. Pairing by `count` gives an MCU↔GPST offset and drift fit at µs level. MCU→EXTINT is PD14 / `GPIO(101)` (user confirmed wired), already toggled by rover mode (`GPS_Rover.cpp:55-58`), which also reports `TIM_TM2` (`:91-93`).

**PPS cross-check** (user confirmed the timepulse reaches the MCU).
- The driver attaches its PPS interrupt only inside the CFG-TP5 reply handler (`AP_GPS_UBLOX.cpp:1370-1379`). With `GPS_AUTO_CONFIG 0`, and no CFG-TP5 on the X20, that handler never runs.
- So AP_Periph attaches its own interrupt on `HAL_GPIO_PPS` (assumed PG2 / GPIO 100) and writes `PPSE: TimeUS(edge), Count`.
- Each X20 TIM-TP (in `UBXR`) gives the GNSS time of that pulse. PPS-derived and strobe-derived offsets must agree to a few µs.
- The replay parser takes the strobe fit as primary and reports the difference as a log-quality metric.

The fallback is the driver's UART-arrival jitter correction (`GPS_Backend.cpp:292-293`), which is ms-level and only good for sanity checks.

**Per-epoch link (required by the replay parser).** Every RAWX epoch also writes `GEPO: TimeUS(frame arrival), RcvTow, Week, LeapS, NumMeas, StrobeFit`. `StrobeFit` is the epoch's MCU time predicted from the latest EXTS↔TM2 fit, with 0 if no fit exists yet. That gives the parser an explicit iTOW↔TimeUS pair per epoch even without reassembling `UBXR`. Raw UBX never lives outside the `.bin` in this design, so a `.ubx` produced offline inherits these pairs.

## Magnetometer Reference

- The RM3100 runs at 80 Hz (`AP_Compass_RM3100.cpp:180-181`).
- The standard `MAG` message is `int16` mGauss, already calibrated (`AP_Logger/LogFile.cpp:400-424`). Note: `Compass::read()` logs on every call while healthy (`AP_Compass.cpp:1752-1755`), and the periph main loop has no delay, so write it from `can_mag_update` only on a new `last_update_ms` (the same guard as `sensor_compass.cpp:32-35`).
- New `MAGF: TimeUS, MagX, MagY, MagZ` is defined exactly as follows:
  - **Type and units:** float, mGauss. The RM3100 driver scales to mGauss: `_scaler = (1/GAIN_CC200) * UTESLA_TO_MGAUSS` (`AP_Compass_RM3100.cpp:158`).
  - **Frame:** board frame. The hook sits in `accumulate_sample` after `rotate_field()` (`AP_Compass_Backend.cpp:143`), at the same point as `publish_raw_field` (`:146`). `rotate_field` applies, in order:
    - `MAG_BOARD_ORIENTATION` (`ROTATION_NONE` on ChibiOS, `AP_Compass.h:27-35`),
    - the hwdef rotation `state.rotation` (`ROTATION_PITCH_180` for the RM3100, `HerePro/hwdef.dat:156`),
    - the AHRS board orientation, because the RM3100 is declared internal (`false` at `:156`), so `COMPASS_ORIENT` is **not** applied. The periph has no AHRS_ORIENTATION, so this is `ROTATION_NONE` (`AP_Compass_Backend.cpp:51-56`).
    - `HAL_HEATER_MAG_OFFSET` is not defined for HerePro.
  - **Corrections:** none. The record is taken before `correct_field()` (`:149`), so offsets, scale factor, diagonals/off-diagonals and motor compensation (`AP_Compass_Backend.cpp:76-136`) are all absent.
  - **Timing:** one record per 80 Hz sensor sample, with no accumulation.
  - **Why:** `int16` resolution in `MAG` (≈0.2° heading at 250 mG horizontal) is too coarse for a truth reference. The hook is a one-line callback in the submodule, gated by `AP_PERIPH_LOGGER`.
- **The RM3100 is the primary heading truth** (user decision). The unit uses the existing suction-cup roof mount (user decision) and is calibrated **in that mounted position**. It sits closer to the roof steel than a mast would, so the in-car calibration circles and the |B|/inclination validity mask carry more weight, and replay's course-over-ground cross-check guards truth quality.
  - The hard/soft-iron fit is done **offline** from the calibration circles in the Field Procedure (2D fit, since a car can't tilt), then corrected for declination with WMM.
  - Start-of-drive and end-of-drive fits are compared as a stability check.
  - The periph `COMPASS_*` values in `PARM` record whatever was applied on board.
- FC motor current and throttle: **N/A for the car**. For later flights, use the FC's own `.bin`, aligned by GPS week/ms. No DroneCAN subscription is added.

## Self-Describing Logs

What a `.bin` carries with no notes:

| Item | Mechanism |
|---|---|
| Firmware string, git hash, board type | AP_Logger startup writer (`LoggerMessageWriter.cpp:280-320`, `VER`) |
| MCU unique ID | startup `sysid` message (`LoggerMessageWriter.cpp:325-330`) |
| Every parameter (BRD_SERIAL_NUM, INS_*, COMPASS_* incl. orientation, GPS_*, LOG_*) | `PARM` at log start |
| X20 hardware/firmware version and extensions | AP_Periph polls MON-VER at each log open; the response is captured in `UBXR` |
| X20 signal plans available and active | Poll MON-GNSS at each log open, captured in `UBXR`. The X20 JSON names SP1/SP2/SP6 but doesn't define them, and the logger leaves the shipped plan in place. The MON-GNSS payload plus a read-only VALGET of `CFG-SIGNAL-*`, `CFG-SIGNAL-PLAN` and `CFG-TP-*` is therefore the only record of what was actually tracked. NAV-SIG at 1 Hz shows the sigIds in use. |
| UBX config in effect | after VALSET, send `CFG-VALGET` (RAM layer) for every key above; the response goes into `UBXR`, plus one `MSG` line `"X20 cfg ok N/N keys"` |
| Logger health | `DSF` (dropped/buffer stats, written by AP_Logger), `UBXR.Seq` gaps, MON-SYS |
| Board layout | `MSG "HereProX-logger IMU rot=… MAG rot=…"` written by a vehicle startup writer (`LoggerMessageWriter.cpp:218-222`) |

## Bandwidth and Storage Budget

The all-band epoch estimate is about 130 measurements in open sky (GPS ~25, GAL ~30, BDS ~40, GLO ~14, QZSS ~8, NavIC/SBAS ~10). RAWX = 16 + 32·n (JSON `sizeStr`) = **4.2 KB per epoch**.

| Stream | Log rate | Log kB/s |
|---|---|---|
| ACC+GYR, 32 B each @ 1 kHz | 2000/s | 64 |
| IMUD ~48 B @ 400 Hz | 400/s | 19 |
| UBXR: RAWX 4.2 KB × 5 Hz | 33 rec/epoch | 24 |
| UBXR: SFRBX (≈3 kB/s raw), PVT, DOP, TM2, TP, NAV-SIG/SAT/MON-RF/SYS @ 1 Hz | — | 9 |
| GPS/GPA/GEPO/EXTS/PPSE, MAG + MAGF @ 80 Hz, PM/DSF/MSG | — | 6 |
| **Total** | | **≈122 kB/s ≈ 440 MB/h** |

- **X20 UART1 load:** about 21 kB/s RAWX + 3 SFRBX + 4 others ≈ 28 kB/s, i.e. 61% of 460800 (46 kB/s at 10 bits per byte) and 30% of 921600.
  - All-band RAWX at 10 Hz is ≈ 42 kB/s by itself, about 90% of 460800.
  - **Choice: 5 Hz RAWX at 921600.** The IMU carries the high-rate dynamics. A 5 Hz pseudorange/carrier update is the normal rate for tight coupling and PPK. 10 Hz would double GNSS log volume and leave only about 40% UART headroom even at 921600.
  - 921600 is chosen over staying at 460800, because 61% leaves no margin for SFRBX bursts after a reacquisition (tunnel exit).
  - Raise to 10 Hz only if replay shows 5 Hz limits the filter.
- **eMMC throughput:** 122 kB/s is far below what SDMMC 8-bit can sustain. The real risk is write-latency stalls (FAT allocation, eMMC garbage collection). `LOG_FILE_BUFSIZE 400` covers about 3 s. The File backend shrinks the buffer if RAM is short and reports it (`AP_Logger_File.cpp:73-90`). Check the boot message and the `DSF` drop counts on the bench run.
- **Capacity:** the bench run measures the eMMC size. Recording hours ≈ (capacity − 0.5 GB `LOG_FILE_MB_FREE` margin, `AP_Logger.cpp:140`) ÷ 0.44 GB/h; for example 4 GB gives about 8 h. Old logs are deleted automatically below that margin.

## Flash and RAM Budget

- **Flash available:** 2048 − 256 (bootloader reserve) − 256 (storage pages 14-15) = **1536 KB** (`HerePro/hwdef.dat:26-30,89,234-235`).
- **Measured:** the Here4 build, which has INS enabled but no logger or scripting, is 319 KB of text (`arm-none-eabi-size build/Here4/bin/AP_Periph`). Of that, AP_InertialSensor symbols are about 16 KB.
- **Estimated:** HerePro adds scripting/base mode, and the logger adds AP_Logger plus the File backend (≈60-90 KB, estimated, not measured). The logger image should land under about 650 KB, comfortably inside 1536 KB.
- **RAM:** the H757 has about 896 KB (`STM32H757xx.py:31-34`). The large items are the logger buffer (400 KB) and the scripting heap (64 KB, `HerePro/hwdef.dat:225`; scripting can be dropped in the logger build if RAM gets tight). The RAWX buffer is about 8 KB.
- **Verify** with `arm-none-eabi-size` and the boot `AP_Logger_File: buffer size=` line on the first build.

## Logging Start and Retrieval

Logging starts at boot with **no arming or CAN dependency**. Parameters in `HereProX-logger/defaults.parm`:

```
LOG_BACKEND_TYPE 1     # file
LOG_DISARMED 2         # log while disarmed, except when USB is detected
LOG_FILE_BUFSIZE 400
LOG_BITMASK <GPS|IMU_RAW|COMPASS bits defined in AP_Periph>
LOG_FILE_DSRMROT 0
CAN_NODE 125           # bypass the DNA gate (can.cpp:720)
GPS_AUTO_CONFIG 0
GPS_RAW_DATA 0
ROVER_ENABLE 0
IMU_SAMPLE_RATE 400
INS_GYRO_RATE 0        # 1 kHz raw IMU (car campaign)
INS_GYR_CAL 0
INS_RAW_LOG_OPT 0
SCR_ENABLE 0
```

- AP_Periph never sets soft-arm, so the logger needs `LOG_DISARMED` (`AP_Logger.cpp:97-102,636`).
- The only field power is USB (a power bank). VBUS alone therefore can't mean "download". The logger build decides by **USB host enumeration**:
  - At boot with VBUS ≥ 4 V, USB + MSD come up as in release, and the device is marked USB-connected, so `LOG_DISARMED 2` doesn't log yet.
  - After a **5 s grace period** (`USB_ENUMERATION_GRACE_MS`), `logger_usb_update()` checks `USBD2.state`:
    - `USB_ACTIVE` (a host has configured the device) → **download mode**: MSD stays on, no logging, purple LED.
    - Anything else → **power bank**: `msdStop`, `usbDisconnectBus`, `clear_usb_connected()` (new `AP_HAL_ChibiOS/GPIO.h` method), and logging starts.
  - Why 5 s: host attach debounce (100 ms) + bus reset + descriptor reads + SET_CONFIGURATION normally finish in under 1 s. 5 s covers slow hubs and costs 5 s of log during the static start.
  - Release's per-loop `set_usb_connected()` re-assert in `update()` is compiled out in the logger build, so the decision holds until reboot.
  - Once logging has started, MSD stays off until reboot. Plugging in a laptop mid-log keeps logging and exposes no drive. To download: unplug, then plug the laptop into USB alone.
- Other power inputs on HerePro (not relied on): `VDD_5V_SENS` on PC4 (5 V rail sense, `HerePro/hwdef.dat:214`) and `ANALOG_CAN_VOLTAGE1/2` on PA1/PA2 (CAN connector voltage sense, `:178-179`). Per the user, the unit can't be powered over CAN in practice.
- This prevents the host and the firmware from writing to the FAT at the same time. MSD also blocks firmware FS access while the host is mounted (`AP_HAL_ChibiOS/sdcard.cpp:34-44,136-146`).
- Logs land in `/APM/LOGS/NNNNNNNN.BIN` (`sdcard.cpp:248` creates `/APM`).

Trigger alternatives (not chosen): DroneCAN arming (`NotifyState` `VEHICLE_STATE_ARMED`, already subscribed at `AP_Periph.h:358-359` and stored at `notify.cpp:50`), an external button on `EXTERN_GPIO2`, or a DroneCAN param/command. All of them fail the standalone car case.

**LED status** (logger build replaces the rainbow/script, `notify.cpp`):

| LED | Meaning |
|---|---|
| green blink 1 Hz | logging, 3D fix |
| blue blink 1 Hz | logging, no fix yet |
| solid red | not logging: no eMMC, write error, X20 config failed, or `DSF` drops in the last 10 s |
| solid purple | USB download mode: a host enumerated within 5 s of boot (not logging); also the first 5 s after power-up on any USB power |

## Field Procedure

**Bench measurement (once, before the first drive)**

The test team fills in the `⟨…⟩` placeholders below from this checklist:

1. **Power connector:** confirm the USB connector type and a power-bank cable that fits → `⟨connector / cable⟩`.
2. **Current draw:** run 10 min from a USB power meter with a 3D fix and logging (green LED) → `⟨mA @ 5 V⟩`. Also confirm the power bank doesn't shut off at that load (some banks cut out below about 100 mA).
3. **eMMC capacity:** connect USB and read the drive size on the laptop → `⟨GB⟩`. Recording hours ≈ (`⟨GB⟩` − 0.5) ÷ 0.44.
4. **Logging check:** a 10-min log shows rising `UBXR.Seq` with no gaps, `DSF` drops = 0, RAWX `numMeas` > 0, and `ACC`/`GYR` at 1 kHz.
5. **Receiver link:** the log shows `MSG "X20 cfg ok N/N keys"` (confirms USART3 ↔ X20 UART1 at 921600).
6. **Timing:**
   - `EXTS` and TIM-TM2 counts increase together at 1 Hz.
   - `PPSE` arrives at 1 Hz (confirms the timepulse is on PG2 / GPIO 100). If there is no `PPSE`, report it. Strobe timing still works.
7. **Battery runtime:** power-bank Wh × 0.8 ÷ (`⟨mA⟩` × 5 V) → `⟨h⟩`. Use a bank that outlasts the planned drive by 50%.

**Before driving**
1. **Power:** a 5 V USB power bank into the unit's USB port (`⟨connector / cable⟩`). The unit stays purple for 5 s, then starts logging. Never start a drive with a laptop attached: a laptop that enumerates puts the unit in download mode (purple, no log). Runtime ≈ `⟨h⟩`; eMMC fits about `⟨recording hours⟩`.
2. **Mounting (the RM3100 is the heading reference):**
   - Use the **existing suction-cup roof mount** from previous tests, on the roof centreline, with clear sky.
   - **No magnetic base, and no added steel near the unit** (no steel brackets, clamps, screws, tools or phone mounts nearby). Keep it away from roof racks, other antennas and cabled devices where possible.
   - Point the unit's arrow toward the car's nose and keep it level.
   - Record: photos from above and from the side, any yaw offset (record "0°" if aligned), and the **height of the unit above the roof surface** → `⟨mm⟩`.
   - If the mount is re-stuck, moved or bumped, **redo the calibration circles** before continuing.
3. Power on while parked in open sky. Within about 60 s the LED goes from blue blinking to green blinking. If the LED is solid red, power-cycle once, then stop and report.
4. **RM3100 calibration on the car:** done with the unit on the suction-cup mount, in an open car park away from parked cars, lamp posts and buried rebar if visible. Drive the calibration circles listed first in the drive plan. Calibration is fitted offline from the log, so nothing needs to be pressed or confirmed on the unit.

**Drive plan (about 75 min)**

| Segment | Duration | Purpose |
|---|---|---|
| Static, open sky | 5 min | GNSS convergence, IMU bias and noise, PPK start |
| Calibration: 3 slow full circles clockwise + 3 counter-clockwise (~10 m radius, open car park), then 2 figure-8s | 5 min | RM3100 in-car fit; heading observability |
| Clean open-sky road (ring road/highway) | 15 min | baseline |
| Overpasses / short bridges, several in a row | 10 min | short outages |
| Tunnel, ≥ 30 s inside (ideally 1-2 min) | — | full blockage |
| Tree-canopy road | 10 min | partial sky |
| Urban canyon (downtown grid) with many 90° turns | 15 min | multipath and partial sky; turns for heading |
| 5 static stops of 30-60 s, at least 2 under cover | spread through | zero-velocity reference |
| Return to open sky: repeat the calibration circles, then 5 min static | 10 min | end-of-run bias and calibration check, PPK end |

Drive normally (no hard manoeuvres needed). Don't stop the logger between segments; one file per drive is fine.

**Position truth (PPK).** Note the start and end location. The static 5 min at the start and end are also there so RTKLIB can resolve ambiguities against a nearby CORS or reference station. Pick a car park within about 20 km of a known CORS if possible.

**Out of scope:** a paired Here4 (F9P) drive (user decision: HereProX only).

**After driving**
1. Power off by unplugging the power bank.
2. Plug the unit's USB into a laptop (nothing else connected). The LED stays purple, and a drive appears. If it turns blue after 5 s, the laptop didn't enumerate it: unplug and try another port or cable. A laptop plugged in while logging does **not** show the drive; always unplug and replug.
3. Copy `APM/LOGS/*.BIN` (the newest numbers) and rename each one as `hereprox-<BRD_SERIAL_NUM>_<YYYY-MM-DD>_<route>_L<NN>.bin`, e.g. `hereprox-1234_2026-11-02_city-loop_L07.bin`. Use the date of the drive. The serial is on the label, and it is also inside the log.
4. Eject the drive before unplugging. Don't delete logs on the unit until they have been uploaded and checked.
5. Add a one-line `notes.txt` (driver, weather, mount photos, anything odd). It's optional, since the log is self-describing.

## Offline Extraction

One script, `scripts/logger/ubxr_to_ubx.py` (pymavlink `DFReader`), turns the `UBXR` records into a `.ubx` file. Its self-check: re-validate every UBX checksum and count `Seq` gaps.

- RTKLIB `convbin` then gives RINEX OBS (from RAWX, all signals via `sigId`) and NAV (from SFRBX).
- RTKLIB `rnx2rtkp` against CORS/IGS data gives post-processed position truth (PPK). IGS broadcast ephemeris (BRDC) is a cross-check if any SFRBX is missing.
- The `EXTS`↔TIM-TM2 pairs give the MCU↔GPST fit, and the `PPSE`↔TIM-TP pairs give the cross-check.

The replay harness owns everything beyond that (see `2026-10-26-replay-harness-design.md`).

## Risks

| Risk | Mitigation |
|---|---|
| Timepulse is not on PG2 / GPIO 100, or USART3 is not X20 UART1 | Bench checklist items 5-6. The strobe stays primary, so a wrong PPS pin only loses the cross-check. |
| X20 rejects or partly applies the VALSET (unknown key) | ACK/NAK captured in `UBXR`, VALGET readback logged, LED red if `ok < N`. Bench-test before handing to the field team. |
| Baud switch to 921600 fails (the driver's binary blob uses legacy CFG-PRT/PUBX, `AP_GPS_UBLOX.h:54-59`) | VALSET `CFG-UART1-BAUDRATE`, re-detection at 921600. Fallback: preconfigure the X20 baud in flash with u-center 2. |
| All-band RAWX over the 80-measurement buffer is silently dropped (`AP_GPS_UBLOX.cpp:793-799`) | `UBLOX_MAX_RXM_RAWX_SATS 255`. Monitor `numMeas` in the first log. |
| Host and firmware writing the eMMC at the same time | `LOG_DISARMED 2`, MSD only with VBUS at boot. |
| eMMC write stalls → dropped data | 400 KB buffer, `DSF` monitoring, LED red on drops. |
| Gyro calibrated while moving | `INS_GYR_CAL 0`. |
| Strobe/TM2 pairing breaks (missed edge, counter wrap) | `count`-based pairing, the PPS/TIM-TP cross-check, then UART jitter correction (ms-level). |
| Car body or mount hardware distorts the RM3100 | No magnetic base or added steel near the unit; calibration in the mounted position at start and end; compare the two fits. |
| The suction-cup mount puts the RM3100 closer to the roof steel than designed (stronger hard/soft iron, more attitude-dependent distortion) | The in-car calibration circles and the \|B\|/inclination validity mask carry more weight. Replay's course-over-ground cross-check (GNSS course at speed vs calibrated RM3100 heading) is the guard on truth quality. Record the mount height so its effect can be compared across drives. |
| Magnetic base, or a laptop attached at power-up in the field | Field Procedure warnings; purple LED that never turns blue = download mode. |
| Host slower than 5 s to enumerate, or a power bank with data lines that look like a host | Bench checks B1a/B1b; `MSG "USB power only…"` in every log records the decision. |
| Product X20 support missing in AP_GPS (MON-VER, VALSET-only, CFG-TP) | Out of scope for the logger. Flag it to the tight-coupling and product workstreams. |

## Decisions Made

1. Raw UBX is stored in the `.bin` (`UBXR`), not in a separate `.ubx` file and not by extending `GRXS`. `GRXH/GRXS` are turned off.
2. AP_Periph configures the X20 by direct VALSET (RAM layer, user OK) with `GPS_AUTO_CONFIG 0`. The driver is not made X20-aware in this workstream.
3. Logging starts at boot (`LOG_DISARMED 2`). USB host enumeration within 5 s = download mode; VBUS without a host = power bank, log. `CAN_NODE 125` gets past the DNA gate.
4. IMU: raw `ACC/GYR` at 1 kHz (car campaign, user decision) plus new `IMUD` delta angles/velocities at 400 Hz. `INS_GYR_CAL 0`.
5. Time sync: a 1 Hz EXTINT strobe with TIM-TM2 (`EXTS` message), plus a per-epoch `GEPO` iTOW↔TimeUS pair. A PPS edge log (`PPSE`) with TIM-TP is the cross-check (user decision).
6. The RM3100 is the primary heading truth: on the existing suction-cup roof mount and calibrated in place, fitted offline from logged calibration circles. New float `MAGF` message.
7. RAWX/SFRBX at 5 Hz, X20 UART at 921600, automotive dynamic model for the car.
8. FC motor current/throttle: not logged by the periph. Later flights use the FC `.bin`, aligned by GPS time.
9. LED states as listed above.
10. The X20 runs its shipped signal plan; the logger only records it. No L6.
11. The logger variant includes `HerePro/hwdef.dat`. The X20 is on USART3 (inferred from the hwdef and defaults, confirmed on the bench).
12. No paired Here4 drive.

## Open Questions

All user questions are answered (2026-10-26). The remaining unknowns are hardware facts the bench checklist resolves:

- Which MCU pin carries the timepulse (assumed PG2 / GPIO 100).
- That USART3 connects to X20 UART1 (and that UART7 is UART2).
- Connector, current draw and eMMC capacity.
