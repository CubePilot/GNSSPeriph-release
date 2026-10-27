# HereProX Logger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `HereProX-logger` firmware variant that records IMU, raw UBX, magnetometer and timing data from boot to eMMC as an ArduPilot `.bin`, plus the field tools to extract it.

**Architecture:** A new board directory includes `HerePro/hwdef.dat` and turns on `HAL_LOGGING_ENABLED`, `AP_INERTIALSENSOR_ENABLED` and `AP_PERIPH_LOGGER`. All new AP_Periph code sits under `#if AP_PERIPH_LOGGER`. All ardupilot-submodule changes sit under `#if AP_GPS_UBLOX_FRAME_CB_ENABLED` / `#if AP_COMPASS_RAW_SAMPLE_CB_ENABLED` (both default 0), so release HerePro builds differ only in version data (git hash, descriptor CRC). The compass hook sits on an existing line so `WITH_SEMAPHORE` `__LINE__` values don't move. With the frame callback enabled, the legacy RAWX-only `_gps_raw_cb` copy is compiled out (its stack buffer scales with `UBLOX_MAX_RXM_RAWX_SATS`), so rover mode's raw file can't be used in the logger build. New log messages use the dynamic `AP::logger().Write*()` API (no message-ID allocation).

**Tech Stack:** ArduPilot AP_Periph (C++, ChibiOS, waf), u-blox UBX over CFG-VALSET, Python 3 + pymavlink for field tools.

**Spec:** `docs/superpowers/specs/2026-10-26-herepro-logger-design.md`

## Global Constraints

- Release HerePro firmware must not change behaviour. New code is behind `AP_PERIPH_LOGGER` (outer repo) or new submodule defines that default to 0.
- Work on branch `herepro-logger` in both the outer repo and `ardupilot/`. Never on `release` / `Here-v1.15`. Leave existing untracked files alone.
- Build: `./waf configure --board HereProX-logger && ./waf AP_Periph`, run in the background.
- Debug prints carry the prefix `DEBUG:`.
- No commits or pushes without asking. Messages are concise, with no qualitative adjectives. Identity: `bugobliterator <siddharthbharatpurohit@gmail.com>` (verify `git config user.email` first).
- Field names convey meaning; no cryptic abbreviations in log labels beyond ArduPilot's 4-char message-name limit.
- Rates from the spec: IMU loop `IMU_SAMPLE_RATE 400`, raw IMU `INS_GYRO_RATE 0` (1 kHz), RAWX 5 Hz, X20 UART1 921600, strobe 1 Hz, `LOG_FILE_BUFSIZE 400`, `CAN_NODE 125`.
- Stop after each phase and report to team-lead (changes, build result, what was verified). The next phase waits for the user.

## Review Focus

1. **USB at boot**: a laptop (host enumerates within 5 s) must give download mode with no log; a power bank (VBUS, no host) must log. Verify on the bench (B1a/B1b). No firmware test can cover it.
2. **RAWX larger than 80 measurements**: the frame must reach `UBXR` intact. Covered by the raw-capture task's buffer-size static assert and the field-tool round-trip with a 150-measurement frame.
3. **UBX frame spanning record boundaries, and records lost mid-frame**: the extractor must drop only the damaged frame and report a `Seq` gap. Covered by `test_ubxr_gap_drops_only_damaged_frame`.
4. **No CAN bus at all**: GPS, compass and IMU still log. Covered by `CAN_NODE 125` in the defaults plus bench check B2.
5. **Strobe edge with no TIM-TM2 reply** (X20 not yet configured): the fit must not use unpaired edges. Covered by `test_strobe_fit_ignores_unpaired`.

---

## Phase: Logger Wiring

Goal of this phase: a flashable `HereProX-logger` image that boots, mounts the eMMC and writes a `.bin` (PARM, VER, MSG, GPS, MAG) from boot with no CAN bus.

### Task: Logger Variant Board

**Files:**
- Create: `HereProX-logger/hwdef.dat`, `HereProX-logger/defaults.parm`
- Modify: `AP_Periph/wscript:24` (add `'HereProX-logger'` to `targets`)

**Interfaces:**
- Produces: the board name `HereProX-logger` and the defines `AP_PERIPH_LOGGER 1`, `HAL_LOGGING_ENABLED 1`, `AP_INERTIALSENSOR_ENABLED 1`, `HAL_LOGGING_FILE_BUFSIZE 400`, `HAL_GPIO_PPS 100`.

- [ ] **Step 1:** Write `HereProX-logger/hwdef.dat` exactly as in the spec's Build Variant block. Leave out the `UBLOX_MAX_RXM_RAWX_SATS` line (added in UBX Raw Capture) and the `HAL_GPIO_PPS` line (added in Time Sync, because `CONFIGURE_PPS_PIN` + `HAL_GPIO_PPS` together would make the driver attach its own interrupt).
- [ ] **Step 2:** Write `HereProX-logger/defaults.parm`: `@include ../HerePro/defaults.parm` plus the spec's parameter block (`LOG_BACKEND_TYPE 1`, `LOG_DISARMED 2`, `LOG_FILE_BUFSIZE 400`, `LOG_BITMASK` = all bits defined in the Logger Object task, `CAN_NODE 125`, `GPS_AUTO_CONFIG 0`, `GPS_RAW_DATA 0`, `ROVER_ENABLE 0`, `IMU_SAMPLE_RATE 400`, `INS_GYRO_RATE 0`, `INS_GYR_CAL 0`, `INS_RAW_LOG_OPT 0`, `SCR_ENABLE 0`).
- [ ] **Step 3:** Add the target to `AP_Periph/wscript`.
- [ ] **Step 4:** Configure: `./waf configure --board HereProX-logger`. Expected: configure succeeds and `ardupilot/libraries/AP_HAL_ChibiOS/hwdef/HereProX-logger` is a symlink.

### Task: Logger Object

**Files:**
- Modify: `AP_Periph/AP_Periph.h` (members), `AP_Periph/Parameters.h` (`AP_Int32 log_bitmask`), `AP_Periph/Parameters.cpp` (`GOBJECT(logger,"LOG",AP_Logger)`, `GSCALAR(log_bitmask,"LOG_BITMASK",…)`), `AP_Periph/AP_Periph.cpp` (`logger.init` at the start of `init()`, `logger.periodic_tasks()` in `update()`, set log bits after `gps.init()` / `compass.init()` / `imu.init()`)

**Interfaces:**
- Produces, in `AP_Periph.h` under `#if AP_PERIPH_LOGGER`:
  - `AP_Logger logger;`
  - `static const struct LogStructure log_structure[];` (= `{ LOG_COMMON_STRUCTURES }`)
  - `enum class LogBit : uint32_t { GPS = 1U<<2, COMPASS = 1U<<3, IMU_RAW = 1U<<4 };`
  - `void write_logger_startup_messages();` (registered with `logger.setVehicle_Startup_Writer`; writes `MSG "HereProX-logger IMU rot=… MAG rot=…"`. More lines are added by later tasks.)
- Consumes: `AP_Logger::init(const AP_Int32&, const LogStructure*, uint8_t)` (`AP_Logger.h:201`), `AP_GPS::set_log_gps_bit`, `Compass::set_log_bit`, `AP_InertialSensor::set_log_raw_bit`.

- [ ] **Step 1:** Add the members, params and calls, all under `#if AP_PERIPH_LOGGER`. Default `LOG_BITMASK` = `GPS|COMPASS|IMU_RAW`.
- [ ] **Step 2:** Build in the background: `./waf AP_Periph`. Expected: `build/HereProX-logger/bin/AP_Periph.apj` exists. Record the `arm-none-eabi-size` text/data/bss.
- [ ] **Step 3:** Release check: `./waf configure --board HerePro && ./waf AP_Periph`. Expected: builds. Compare the `.bin` against a pre-change HerePro build of the same commit (use a release build captured before the first edit) and expect an identical size.
- [ ] **Step 4:** Report the phase. Bench checks for the test team: B1 (USB at boot → no new log, purple later), B2 (no CAN, battery power → `.bin` appears in `/APM/LOGS`, PARM present).

---

## Phase: Core Capture

Goal of this phase: the same image now also records 1 kHz `ACC/GYR`, 400 Hz `IMUD`, `MAGF` and every UBX frame as `UBXR`. This is the first build worth giving the test team (once X20 Config lands).

### Task: IMU Logging

**Files:**
- Modify: `AP_Periph/sensor_imu.cpp:10-39`

**Interfaces:**
- Produces log message `IMUD`: labels `TimeUS,DelAngX,DelAngY,DelAngZ,DelAngDT,DelVelX,DelVelY,DelVelZ,DelVelDT,Temp`, format `Qffffffffff`, units `srrrsnnnsO`, written with `AP::logger().WriteStreaming` once per new `imu.get_last_update_usec()`.

- [ ] **Step 1:** Under `#if AP_PERIPH_LOGGER`:
  - write `IMUD` after each new sample,
  - replace the `return` on a repeated timestamp (`:18-19`) with `continue`,
  - store `periph.last_imu_update_usec`,
  - skip `raw_imu_pub.broadcast`.

  The release path stays unchanged.
- [ ] **Step 2:** Build. Expected: success.
- [ ] **Step 3:** Bench check B3: a log contains `IMUD` at about 400 Hz and `ACC`/`GYR` at about 1000 Hz (measure from TimeUS deltas in MAVExplorer).

### Task: MAGF Raw Sample Hook

**Files:**
- Modify (submodule): `ardupilot/libraries/AP_Compass/AP_Compass_Backend.cpp:143-149`, `AP_Compass.h` (callback member + setter)
- Modify: `AP_Periph/sensor_compass.cpp` or `AP_Periph/AP_Periph.cpp` (register callback)

**Interfaces:**
- Produces in `Compass` (under `#if AP_COMPASS_RAW_SAMPLE_CB_ENABLED`, default 0, set to 1 in the logger hwdef):
  - `FUNCTOR_TYPEDEF(raw_sample_cb_t, void, uint8_t instance, const Vector3f &field_board_frame_mgauss);`
  - `void set_raw_sample_cb(raw_sample_cb_t cb);`
  - Called in `accumulate_sample` after `rotate_field()` and before `correct_field()`.
- Produces log message `MAGF`: labels `TimeUS,I,MagX,MagY,MagZ`, format `QBfff`, units `s#GGG`. Definition (spec): board frame, no calibration corrections, float mGauss, one record per 80 Hz sample.

- [ ] **Step 1:** Add the callback in the submodule behind the new define.
- [ ] **Step 2:** Register it in AP_Periph under `#if AP_PERIPH_LOGGER`. The callback writes `MAGF` via `WriteStreaming`.
- [ ] **Step 3:** Fix the `MAG` rate: `Compass::read()` logs on every call (`AP_Compass.cpp:1752-1755`). Set the compass log bit only in the logger build, and write `MAG` from `can_mag_update` guarded by a new `last_update_ms` (the same guard as `sensor_compass.cpp:32-35`), instead of relying on `Compass::read()`'s logging.
- [ ] **Step 4:** Build both boards. Expected: success. `HerePro` size unchanged.

### Task: UBX Raw Capture

**Files:**
- Modify (submodule): `ardupilot/libraries/AP_GPS/AP_GPS_UBLOX.h:63` (`#ifndef UBLOX_MAX_RXM_RAWX_SATS`), `AP_GPS_UBLOX.cpp:841` (frame hook), `AP_GPS_UBLOX.cpp:1050-1061` (gate auto-disable on `GPS_AUTO_CONFIG`), `AP_GPS.h` (frame callback member + setter)
- Create: `AP_Periph/logger_ubx.cpp` (UBXR writer)
- Modify: `AP_Periph/AP_Periph.h`, `AP_Periph/wscript` (if the source glob needs it), `HereProX-logger/hwdef.dat` (`define UBLOX_MAX_RXM_RAWX_SATS 255`, `define AP_GPS_UBLOX_FRAME_CB_ENABLED 1`)

**Interfaces:**
- Produces in `AP_GPS` (under `#if AP_GPS_UBLOX_FRAME_CB_ENABLED`):
  - `FUNCTOR_TYPEDEF(ubx_frame_cb_t, void, uint8_t instance, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);` (payload only; no 8 KB stack copy),
  - `void set_ubx_frame_cb(ubx_frame_cb_t cb);`
- The driver rebuilds the frame from `_class`, `_msg_id`, `_payload_length`, `_buffer`, `_ck_a` and `_ck_b` at `AP_GPS_UBLOX.cpp:841`, before `_parse_gps()`.
- Produces in AP_Periph: `void AP_Periph_FW::log_ubx_frame(uint8_t instance, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);` (re-frames and streams into UBXR records)
  - Writes `UBXR`: labels `TimeUS,Seq,Len,Data0,Data1`, format `QHBaa`.
  - `Len` = valid bytes in this record (1..128).
  - `Seq` = a uint16 counter shared by all records, wrapping.
  - A frame is split into consecutive records of 128 bytes; the last record's `Len` is the remainder.
- Static assert in `logger_ubx.cpp`: `sizeof(AP_GPS_UBLOX::ubx_rxm_rawx)` ≥ 16 + 32*150 when `UBLOX_MAX_RXM_RAWX_SATS` = 255. (If the type is private, assert on `UBLOX_MAX_RXM_RAWX_SATS >= 150` instead.)

- [ ] **Step 1:** Write the submodule changes behind the defines.
- [ ] **Step 2:** Write `log_ubx_frame` and register it after `gps.init()`.
- [ ] **Step 3:** Build both boards. Expected: success. `HerePro` size unchanged.
- [ ] **Step 4:** Report the phase. Bench check B4: `UBXR` records appear once the X20 outputs UBX. Until X20 Config lands, only whatever the X20 emits by default (possibly nothing UBX).

### Task: UBXR Extractor (Field Tools)

**Files:**
- Create: `scripts/logger/ubxr_to_ubx.py`, `scripts/logger/test_ubxr_to_ubx.py`

**Interfaces:**
- `ubxr_records_to_frames(records: Iterable[tuple[int, int, int, bytes]]) -> tuple[list[bytes], int]`
  - Input: `(time_us, seq, length, data128)` tuples.
  - Returns: (valid UBX frames in order, count of `Seq` gaps).
- `split_frame(frame: bytes, seq0: int) -> list[tuple[int, int, bytes]]`: the Python mirror of the firmware chunking, used by tests.
- CLI: `python3 scripts/logger/ubxr_to_ubx.py LOG.bin -o LOG.ubx`. It prints the frame count, the gap count and the counts per class/id.

- [ ] **Step 1: Write the failing tests** in `test_ubxr_to_ubx.py` (plain `unittest`, no fixtures):
  - `test_round_trip`: synthetic stream of NAV-PVT (92-byte payload), RAWX with 150 measurements (4816-byte payload), SFRBX (8 + 16*4) and TIM-TM2 (28) → `split_frame` → `ubxr_records_to_frames` → byte-identical frames, 0 gaps.
  - `test_ubxr_gap_drops_only_damaged_frame`: drop one middle record of the RAWX frame → that frame is missing, the other three survive, gaps == 1.
  - `test_seq_wrap`: start `Seq` at 65530 across a RAWX frame → 0 gaps.
- [ ] **Step 2:** Run `python3 -m unittest scripts/logger/test_ubxr_to_ubx.py -v`. Expected: FAIL (module missing).
- [ ] **Step 3:** Implement. Reassemble by scanning for `0xB5 0x62`, reading the length, verifying the Fletcher checksum, and resyncing after a gap. Read `.bin` with `pymavlink.DFReader.DFReader_binary`, and turn `Data0`/`Data1` (lists of 32 int16) back into bytes with `struct.pack('<32h', …)`.
- [ ] **Step 4:** Run the tests. Expected: 3 passed.

---

## Phase: X20 Config

Goal of this phase: the logger puts the X20 into the spec's message set at 921600 on every boot and records proof of the config in the log. After this phase the test team can drive.

### Task: X20 VALSET Sequencer

**Files:**
- Create: `AP_Periph/logger_x20.cpp`
- Modify: `AP_Periph/AP_Periph.h`, `AP_Periph/AP_Periph.cpp` (call from `update()`)

**Interfaces:**
- `void AP_Periph_FW::x20_config_update();`, called from `update()`. It is a state machine:
  - `WAIT_DETECT` (until `gps.status() != NO_GPS` or 5 s after a UBX frame was seen),
  - `SEND_VALSET` (RAM layer 0x01, keys and values from the spec table, without the baud key),
  - `WAIT_ACK` (ACK-ACK class 0x05 id 0x01 for CFG 0x06/0x8A, seen through the frame callback; resend every 1 s, at most 10 tries),
  - `SEND_BAUD` (`CFG-UART1-BAUDRATE` 0x40520001 = 921600; no ACK expected at the old rate),
  - `WAIT_REDETECT`,
  - `READBACK` (one CFG-VALGET poll, RAM layer, for every key written plus `CFG-SIGNAL` group wildcard 0x1031ffff, `CFG-SIGNAL-PLAN` 0x2031003a and `CFG-TP` wildcard 0x1005ffff),
  - `POLL_INFO` (MON-VER 0x0A/0x04 and MON-GNSS 0x0A/0x28, both empty-payload polls),
  - `DONE` or `FAILED`.
- Sends go through `AP::gps().inject_data(0, buf, len)` (`AP_GPS.h:761`).
- Writes `MSG "X20 cfg ok N/N keys"` or `MSG "X20 cfg FAILED step=<name>"`.
- Exposes `bool x20_config_ok() const;` for the status LED.
- `uint16_t ubx_build(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_size);` builds a framed message and returns its length (0 if it doesn't fit).
- `void x20_on_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);` is called from `log_ubx_frame` and handles ACK/NAK.
- The key table is one `static const` array of `{uint32_t key; uint32_t value;}`. The value width comes from key bits 28-30 (size code 1→1 byte (L), 2→1, 3→2, 4→4).
- MON-VER and MON-GNSS are also re-polled at every log file open. Hook: the vehicle startup writer.

- [ ] **Step 1:** Write `ubx_build` and the key table. The X20's ACK/NAK is the test: a wrong frame gets no ACK (bench check B5), and the log records which step failed. No host test, because a Python re-encoder would only test itself.
- [ ] **Step 2:** Write the state machine.
- [ ] **Step 3:** Build both boards. Expected: success. `HerePro` size unchanged.
- [ ] **Step 4:** Report the phase. Bench checks:
  - B5: `MSG "X20 cfg ok"`.
  - B6: `UBXR` contains RAWX (0x02/0x15), SFRBX (0x02/0x13), PVT, MON-RF, TIM-TM2, the VALGET response, MON-VER and MON-GNSS.
  - B7: `GPS` messages show a 3D fix at 5 Hz.

---

## Phase: Time Sync

### Task: EXTINT Strobe, PPS Edges and Per-Epoch Link

**Files:**
- Create: `AP_Periph/logger_timesync.cpp`
- Modify: `AP_Periph/AP_Periph.h`, `AP_Periph/AP_Periph.cpp`, `AP_Periph/logger_ubx.cpp` (call `log_epoch_link` on RAWX), `HereProX-logger/hwdef.dat` (`define HAL_GPIO_PPS 100`, `undef CONFIGURE_PPS_PIN` + `define CONFIGURE_PPS_PIN 0` so the driver doesn't attach its own handler)
- Create: `scripts/logger/timesync_fit.py`, `scripts/logger/test_timesync_fit.py`

**Interfaces:**
- `void AP_Periph_FW::strobe_update();`, called from `update()`:
  - every 1000 ms: `chSysLock(); t = AP_HAL::micros64(); hal.gpio->write(GPS_EXTINT, 1); chSysUnlock();`, then write `EXTS` (labels `TimeUS,Count`, format `QI`),
  - 10 ms later: write 0.
- `void AP_Periph_FW::pps_edge_isr(uint8_t pin, bool high, uint32_t timestamp_us);` is attached with `hal.gpio->attach_interrupt(HAL_GPIO_PPS, …, INTERRUPT_RISING)`. It queues the edge time (`ObjectBuffer<uint64_t>` of 8), and `update()` drains the queue to `PPSE` (labels `TimeUS,Count`, format `QI`).
- `void AP_Periph_FW::log_epoch_link(const uint8_t *rawx_frame, uint16_t len, uint64_t arrival_us);` writes `GEPO` (labels `TimeUS,RcvTow,Week,LeapS,NumMeas,StrobeFit`, format `QdHbBQ`).
  - `StrobeFit` = predicted MCU µs for `RcvTow` from a running linear fit over the last 10 EXTS↔TIM-TM2 pairs. TIM-TM2 is paired with an EXTS record by TM2 `count`; the firmware keeps `Count` equal to the strobe rising-edge number since boot, and the pair is aligned on the first TM2 seen.
  - `StrobeFit` = 0 until 2 pairs exist.
- `timesync_fit.py`: `fit_mcu_to_gpst(exts, tm2) -> (offset_s, drift_ppm, rms_us, pairs)`. Arguments: `exts` = (time_us, count), `tm2` = (arrival_time_us, count, week, tow_ms, tow_sub_ns). Each TIM-TM2 is paired with the latest EXTS edge before its arrival (within 0.9 s), because the receiver's edge counter and the MCU counter start at different times. `offset_s` is at the mid-point, in GPS seconds from the start of the first mark's week (absolute GPS seconds would lose double precision at ~0.2 µs).

- [ ] **Step 1: Write the failing tests** in `test_timesync_fit.py`:
  - `test_fit_recovers_offset_and_drift`: synthetic offset 1234.5 s, drift 12 ppm, 1 µs noise → recovered within 0.1 µs offset at the mid-point and 0.05 ppm.
  - `test_strobe_fit_ignores_unpaired`: EXTS counts 1..20, TM2 only for 5..20 → fit uses 16 pairs, same result.
- [ ] **Step 2:** Run `python3 -m unittest scripts/logger/test_timesync_fit.py -v`. Expected: FAIL.
- [ ] **Step 3:** Implement `timesync_fit.py` (least squares with numpy if installed, otherwise a closed-form 2-parameter fit).
- [ ] **Step 4:** Run the tests. Expected: 2 passed.
- [ ] **Step 5:** Implement the firmware pieces. Build both boards. Expected: success. `HerePro` size unchanged.
- [ ] **Step 6:** Report the phase. Bench checks:
  - B8: `EXTS` at 1 Hz, TIM-TM2 count tracks it.
  - B9: `PPSE` at 1 Hz (confirms PG2 / GPIO 100).
  - B10: `timesync_fit.py` on a bench log gives rms < 5 µs, and PPS-derived offset within 5 µs of strobe-derived.

---

## Phase: Field Readiness

### Task: Status LED

**Files:**
- Modify: `AP_Periph/notify.cpp` (`update_rainbow`, under `#if AP_PERIPH_LOGGER`)

**Interfaces:**
- Consumes `logger.logging_started()`, `logger.logging_failed()`, `gps.status()`, `x20_config_ok()`, `hal.gpio->usb_connected()`, and `DSF` drop counts via `logger.num_dropped()`.
- LED states exactly as in the spec table:
  - green blink 1 Hz = logging + 3D fix,
  - blue blink 1 Hz = logging, no fix,
  - solid red = no eMMC / write error / X20 config failed / drops in the last 10 s,
  - solid purple = USB.

- [ ] **Step 1:** Implement.
- [ ] **Step 2:** Build both boards.
- [ ] **Step 3:** Bench check B11: each LED state, provoked where possible (USB, no fix indoors, eMMC full is not provokable → skip).

### Task: Field Procedure Hand-off

**Files:**
- Modify: `docs/superpowers/specs/2026-10-26-herepro-logger-design.md` (fill in the bench-check numbers once measured)
- Create: `scripts/logger/README.md`. The user asked for the field tools, so this documents extractor and fit usage in 10 lines.

- [ ] **Step 1:** Write the README.
- [ ] **Step 2:** Run all Python checks: `python3 -m unittest discover -s scripts/logger -v`. Expected: all pass.
- [ ] **Step 3:** Report: a final build of both boards, sizes, and the list of bench checks B1-B11 still open.

---

## Phase: USB Power Mode

The only field power is USB. VBUS ≥ 4 V at boot used to mean download mode for good, so a power bank logged nothing.

### Task: Decide USB Mode by Host Enumeration

**Files:**
- Modify (submodule): `ardupilot/libraries/AP_HAL_ChibiOS/GPIO.h` (`void clear_usb_connected()`)
- Modify: `AP_Periph/AP_Periph.h`, `AP_Periph/AP_Periph.cpp` (all under `#if AP_PERIPH_LOGGER`)

**Interfaces:**
- `enum class UsbMode { NO_VBUS, PENDING, DOWNLOAD, POWER_ONLY }`, `USB_ENUMERATION_GRACE_MS = 5000`, `void logger_usb_update();`.
- The VBUS branch of `init()` sets `PENDING`. `update()` calls `logger_usb_update()` instead of re-asserting `set_usb_connected()`.
- After the grace period: `USBD2.state == USB_ACTIVE` → `DOWNLOAD`. Otherwise → `msdStop`, `usbDisconnectBus`, `clear_usb_connected()`, `POWER_ONLY`.
- The startup `MSG "USB …"` records the mode.

- [x] **Step 1:** Implement.
- [x] **Step 2:** Build both boards; release-identity check.
- [ ] **Step 3:** Bench B1a (laptop → purple, drive visible, no new log) and B1b (power bank → purple 5 s, then blue/green, `.bin` grows, laptop plugged in mid-log shows no drive).
