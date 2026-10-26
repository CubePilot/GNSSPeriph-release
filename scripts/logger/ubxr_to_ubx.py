#!/usr/bin/env python3
"""
Extract raw u-blox frames from HereProX-logger UBXR records into a .ubx file.

  python3 scripts/logger/ubxr_to_ubx.py 00000007.BIN -o drive.ubx

The firmware writes each UBX frame as consecutive UBXR records
(TimeUS, Seq, Len, Data0, Data1), each record carrying up to 128 bytes;
a frame always starts at a record boundary. Seq increments per record,
so a missing record is detected and only the damaged frame is dropped.
"""
import argparse
import collections
import struct
import sys

SYNC = b'\xb5\x62'
RECORD_BYTES = 128


def ubx_checksum(body):
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def ubx_frame(msg_class, msg_id, payload):
    body = struct.pack('<BBH', msg_class, msg_id, len(payload)) + payload
    return SYNC + body + ubx_checksum(body)


def split_frame(frame, seq0):
    """mirror of the firmware chunking: list of (time_us, seq, length, data128)"""
    records = []
    seq = seq0
    for i in range(0, len(frame), RECORD_BYTES):
        chunk = frame[i:i + RECORD_BYTES]
        records.append((0, seq, len(chunk), chunk.ljust(RECORD_BYTES, b'\0')))
        seq = (seq + 1) & 0xFFFF
    return records


def ubxr_records_to_frames(records):
    """records: iterable of (time_us, seq, length, data128) -> (frames, seq_gaps)"""
    frames = []
    gaps = 0
    buf = bytearray()
    expected_seq = None
    resync = False
    for _time_us, seq, length, data in records:
        if expected_seq is not None and seq != expected_seq:
            gaps += 1
            buf.clear()
            resync = True
        expected_seq = (seq + 1) & 0xFFFF
        chunk = bytes(data[:length])
        if resync:
            if not chunk.startswith(SYNC):
                continue
            resync = False
        buf += chunk
        while len(buf) >= 8:
            if not buf.startswith(SYNC):
                buf.clear()
                resync = True
                break
            payload_len = struct.unpack_from('<H', buf, 4)[0]
            frame_len = 8 + payload_len
            if len(buf) < frame_len:
                break
            frame = bytes(buf[:frame_len])
            del buf[:frame_len]
            if frame[-2:] == ubx_checksum(frame[2:-2]):
                frames.append(frame)
            else:
                buf.clear()
                resync = True
                break
    return frames, gaps


def read_ubxr_records(bin_path):
    from pymavlink import DFReader
    log = DFReader.DFReader_binary(bin_path, zero_time_base=True)
    while True:
        m = log.recv_match(type='UBXR')
        if m is None:
            return
        data = struct.pack('<32h', *m.Data0) + struct.pack('<32h', *m.Data1)
        yield (m.TimeUS, m.Seq, m.Len, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bin', help='HereProX-logger .bin log')
    parser.add_argument('-o', '--output', help='output .ubx (default: <bin>.ubx)')
    args = parser.parse_args()

    frames, gaps = ubxr_records_to_frames(read_ubxr_records(args.bin))
    out_path = args.output or args.bin.rsplit('.', 1)[0] + '.ubx'
    with open(out_path, 'wb') as f:
        for frame in frames:
            f.write(frame)

    counts = collections.Counter((fr[2], fr[3]) for fr in frames)
    print('%s: %u frames, %u Seq gaps' % (out_path, len(frames), gaps))
    for (cls, mid), n in sorted(counts.items()):
        print('  0x%02X 0x%02X  %u' % (cls, mid, n))
    return 0 if frames else 1


if __name__ == '__main__':
    sys.exit(main())
