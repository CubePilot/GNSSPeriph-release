#!/usr/bin/env python3
"""
Fit the HereProX-logger MCU clock (TimeUS) to GPS time.

  python3 scripts/logger/timesync_fit.py 00000007.BIN

Primary: EXTS (MCU time of each EXTINT strobe edge) paired with the
X20's UBX-TIM-TM2 (GPS time of the same edge). Each TIM-TM2 is paired
with the latest EXTS edge before its arrival (within 0.9 s); unpaired
edges are ignored. Cross-check: PPSE (MCU time of each timepulse edge)
paired with UBX-TIM-TP (GPS time of the next pulse, sent before it).
"""
import argparse
import math
import struct
import sys

WEEK_S = 604800
PAIR_WINDOW_US = 900_000


def linear_fit(xs, ys):
    """least-squares y = a + b*x on values relative to the first sample; returns (x0, y0, a, b, rms)"""
    x0, y0 = xs[0], ys[0]
    dx = [x - x0 for x in xs]
    dy = [y - y0 for y in ys]
    n = len(dx)
    mx, my = sum(dx) / n, sum(dy) / n
    sxx = sum((x - mx) ** 2 for x in dx)
    sxy = sum((x - mx) * (y - my) for x, y in zip(dx, dy))
    b = sxy / sxx if sxx > 0 else 1.0
    a = my - b * mx
    rms = math.sqrt(sum((y - (a + b * x)) ** 2 for x, y in zip(dx, dy)) / n)
    return x0, y0, a, b, rms


def pair_latest_before(edges_us, marks):
    """marks: (arrival_us, gps_s); pair each with the latest edge before arrival within the window"""
    pairs = []
    edges = sorted(edges_us)
    i = 0
    for arrival_us, gps_s in sorted(marks):
        while i + 1 < len(edges) and edges[i + 1] <= arrival_us:
            i += 1
        if edges and edges[i] <= arrival_us and arrival_us - edges[i] <= PAIR_WINDOW_US:
            pairs.append((edges[i], gps_s))
    # one mark per edge
    unique = {}
    for mcu_us, gps_s in pairs:
        unique.setdefault(mcu_us, gps_s)
    return sorted(unique.items())


def fit_pairs(pairs):
    """pairs: (mcu_us, gps_s) -> (offset_s at mid-point, drift_ppm, rms_us, n)"""
    if len(pairs) < 2:
        raise ValueError('need at least 2 paired edges, got %u' % len(pairs))
    xs = [p[0] * 1e-6 for p in pairs]
    ys = [p[1] for p in pairs]
    x0, y0, a, b, rms = linear_fit(xs, ys)
    mid = (min(xs) + max(xs)) / 2
    gps_mid = y0 + a + b * (mid - x0)
    return gps_mid - mid, (b - 1.0) * 1e6, rms * 1e6, len(pairs)


def fit_mcu_to_gpst(exts, tm2):
    """
    exts: (time_us, count) per strobe edge
    tm2:  (arrival_time_us, count, week, tow_ms, tow_sub_ns) per UBX-TIM-TM2
    returns (offset_s, drift_ppm, rms_us, pairs) with gps_s = mcu_s + offset_s at the mid-point,
    gps_s counted from the start of the first time mark's GPS week (keeps double precision at ns level)
    """
    week0 = tm2[0][2] if tm2 else 0
    marks = [(arrival, (week - week0) * WEEK_S + tow_ms * 1e-3 + sub_ns * 1e-9)
             for arrival, _count, week, tow_ms, sub_ns in tm2]
    return fit_pairs(pair_latest_before([e[0] for e in exts], marks))


def fit_pps(ppse, tim_tp):
    """ppse: (time_us, count); tim_tp: (arrival_us, week, tow_ms, tow_sub_ms_2e32). TIM-TP precedes its pulse."""
    edges = sorted(p[0] for p in ppse)
    pairs = []
    week0 = min(t[1] for t in tim_tp)
    for arrival, week, tow_ms, sub in sorted(tim_tp):
        nxt = [e for e in edges if arrival < e <= arrival + 1_100_000]
        if nxt:
            pairs.append((nxt[0], (week - week0) * WEEK_S + tow_ms * 1e-3 + sub * 2.0 ** -32 * 1e-3))
    return fit_pairs(sorted(dict(pairs).items()))


def read_log(bin_path):
    from pymavlink import DFReader
    from ubxr_to_ubx import ubxr_records_to_frames, read_ubxr_records
    log = DFReader.DFReader_binary(bin_path, zero_time_base=True)
    exts, ppse = [], []
    while True:
        m = log.recv_match(type=['EXTS', 'PPSE'])
        if m is None:
            break
        (exts if m.get_type() == 'EXTS' else ppse).append((m.TimeUS, m.Count))
    tm2, tim_tp = [], []
    timed_frames, _gaps = ubxr_records_to_frames(read_ubxr_records(bin_path), with_times=True)
    for time_us, frame in timed_frames:
        cls, mid, payload = frame[2], frame[3], frame[6:-2]
        if (cls, mid) == (0x0D, 0x03) and len(payload) == 28:
            _ch, _flags, count, wn_r, _wn_f, tow_ms_r, tow_sub_ns_r = struct.unpack_from('<BBHHHII', payload)
            tm2.append((time_us, count, wn_r, tow_ms_r, tow_sub_ns_r))
        elif (cls, mid) == (0x0D, 0x01) and len(payload) == 16:
            tow_ms, tow_sub, _qerr, week = struct.unpack_from('<IIiH', payload)
            tim_tp.append((time_us, week, tow_ms, tow_sub))
    return exts, tm2, ppse, tim_tp


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bin', help='HereProX-logger .bin log')
    args = parser.parse_args()
    exts, tm2, ppse, tim_tp = read_log(args.bin)
    offset_s, drift_ppm, rms_us, n = fit_mcu_to_gpst(exts, tm2)
    print('strobe: %u pairs  offset %.6f s  drift %.3f ppm  rms %.2f us' % (n, offset_s, drift_ppm, rms_us))
    if ppse and tim_tp:
        p_offset, p_drift, p_rms, p_n = fit_pps(ppse, tim_tp)
        print('pps:    %u pairs  offset %.6f s  drift %.3f ppm  rms %.2f us' % (p_n, p_offset, p_drift, p_rms))
        print('strobe - pps offset at mid-points: %.2f us' % ((offset_s - p_offset) * 1e6))
    else:
        print('pps: no PPSE/TIM-TP pairs')
    return 0


if __name__ == '__main__':
    sys.exit(main())
