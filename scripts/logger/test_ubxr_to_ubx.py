#!/usr/bin/env python3
"""self-check for ubxr_to_ubx: python3 -m unittest scripts/logger/test_ubxr_to_ubx.py -v"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ubxr_to_ubx import ubx_frame, split_frame, ubxr_records_to_frames  # noqa: E402


def synthetic_frames():
    rng = random.Random(1)

    def payload(n):
        return bytes(rng.randrange(256) for _ in range(n))

    return [
        ubx_frame(0x01, 0x07, payload(92)),               # NAV-PVT
        ubx_frame(0x02, 0x15, payload(16 + 32 * 150)),    # RAWX, 150 measurements
        ubx_frame(0x02, 0x13, payload(8 + 16 * 4)),       # SFRBX, 16 words
        ubx_frame(0x0D, 0x03, payload(28)),               # TIM-TM2
    ]


def records_for(frames, seq0=0):
    records = []
    seq = seq0
    for frame in frames:
        chunks = split_frame(frame, seq)
        records.extend(chunks)
        seq = (seq + len(chunks)) & 0xFFFF
    return records


class UBXRTest(unittest.TestCase):
    def test_round_trip(self):
        frames = synthetic_frames()
        out, gaps = ubxr_records_to_frames(records_for(frames))
        self.assertEqual(out, frames)
        self.assertEqual(gaps, 0)

    def test_ubxr_gap_drops_only_damaged_frame(self):
        frames = synthetic_frames()
        records = records_for(frames)
        rawx_first = len(split_frame(frames[0], 0))
        del records[rawx_first + 10]          # a record in the middle of RAWX
        out, gaps = ubxr_records_to_frames(records)
        self.assertEqual(out, [frames[0], frames[2], frames[3]])
        self.assertEqual(gaps, 1)

    def test_frame_times(self):
        frames = synthetic_frames()
        records = [(1000 * i, seq, n, d) for i, (_t, seq, n, d) in enumerate(records_for(frames))]
        out, _gaps = ubxr_records_to_frames(records, with_times=True)
        first_record_of_frame = [0]
        for fr in frames[:-1]:
            first_record_of_frame.append(first_record_of_frame[-1] + len(split_frame(fr, 0)))
        self.assertEqual([t for t, _f in out], [1000 * i for i in first_record_of_frame])
        self.assertEqual([f for _t, f in out], frames)

    def test_seq_wrap(self):
        frames = synthetic_frames()
        out, gaps = ubxr_records_to_frames(records_for(frames, seq0=65530))
        self.assertEqual(out, frames)
        self.assertEqual(gaps, 0)


if __name__ == '__main__':
    unittest.main()
