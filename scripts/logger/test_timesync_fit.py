#!/usr/bin/env python3
"""self-check for timesync_fit: python3 -m unittest scripts/logger/test_timesync_fit.py -v"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timesync_fit import fit_mcu_to_gpst  # noqa: E402

OFFSET_S = 1234.5
DRIFT_PPM = 12.0
WEEK = 2390


def synthetic(edges, tm2_for, noise_us=1.0, seed=2):
    """EXTS edges once a second from t=100 s; TIM-TM2 for the edge numbers in tm2_for, arriving 150 ms later"""
    rng = random.Random(seed)
    exts, tm2 = [], []
    for n in edges:
        mcu_us = 100_000_000 + (n - 1) * 1_000_000
        exts.append((mcu_us, n))
        if n in tm2_for:
            mcu_s = mcu_us * 1e-6
            gps_s = mcu_s + OFFSET_S + DRIFT_PPM * 1e-6 * mcu_s + rng.gauss(0, noise_us * 1e-6)
            tow_s = gps_s % 604800
            tow_ms = int(tow_s * 1000)
            tow_sub_ns = int(round((tow_s * 1000 - tow_ms) * 1e6))
            # receiver edge counter starts at 7000, unrelated to the MCU count
            tm2.append((mcu_us + 150_000, 7000 + n, WEEK, tow_ms, tow_sub_ns))
    return exts, tm2


class TimesyncFitTest(unittest.TestCase):
    def check(self, exts, tm2, paired_mcu_us):
        offset_s, drift_ppm, rms_us, pairs = fit_mcu_to_gpst(exts, tm2)
        mid_s = (min(paired_mcu_us) + max(paired_mcu_us)) * 0.5e-6
        gps_mid_expected = mid_s + OFFSET_S + DRIFT_PPM * 1e-6 * mid_s
        gps_mid_fit = mid_s + offset_s
        # fit reports GPS seconds from the start of the first mark's week
        self.assertAlmostEqual(gps_mid_fit - gps_mid_expected, 0.0, delta=0.1e-6)
        self.assertAlmostEqual(drift_ppm, DRIFT_PPM, delta=0.05)
        self.assertLess(rms_us, 3.0)
        self.assertEqual(pairs, len(paired_mcu_us))

    def test_fit_recovers_offset_and_drift(self):
        exts, tm2 = synthetic(range(1, 601), set(range(1, 601)))
        self.check(exts, tm2, [e[0] for e in exts])

    def test_strobe_fit_ignores_unpaired(self):
        exts, tm2 = synthetic(range(1, 21), set(range(5, 21)), noise_us=0.0)
        self.check(exts, tm2, [e[0] for e in exts if e[1] >= 5])


if __name__ == '__main__':
    unittest.main()
