"""
Verify a .csi capture: read every record back and report what is actually in it.

Run this right after a capture, BEFORE trusting the session. It answers "did I
really record usable data" without needing the analysis pipeline, and catches the
failure modes that silently ruin a session: gaps, mixed frame widths, all-zero
I/Q, and frame rate that differs between conditions.

    python csi_verify.py data/session1.csi
"""

import argparse
import json
import os
import struct
import sys

MAGIC = 0xC5110003
HEADER_FMT = "<IBBHIQbbBBBBBBBBBBBBBBBB6sH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

HEADER_FIELDS = [
    "magic", "node_id", "num_antennas", "num_subcarriers", "sequence",
    "timestamp_us", "rssi", "noise_floor", "channel", "secondary_channel",
    "rate", "sig_mode", "mcs", "cwb", "smoothing", "not_sounding",
    "aggregation", "stbc", "fec_coding", "sgi", "ampdu_cnt", "rx_state",
    "first_word_invalid", "phy_variant", "mac", "csi_len",
]


def read_records(path):
    """Yield (recv_unix, label, header_dict, csi_bytes) per record."""
    with open(path, "rb") as f:
        while True:
            raw_len = f.read(4)
            if len(raw_len) < 4:
                return
            (n,) = struct.unpack("<I", raw_len)
            rec = f.read(n)
            if len(rec) < n:
                print(f"  !! truncated final record ({len(rec)}/{n} bytes) -- "
                      f"ignoring it; the rest of the file is fine")
                return
            recv, lbl_len = struct.unpack_from("<dB", rec, 0)
            off = 9
            label = rec[off:off + lbl_len].decode("utf-8", "replace")
            off += lbl_len
            hdr = dict(zip(HEADER_FIELDS, struct.unpack_from(HEADER_FMT, rec, off)))
            csi = rec[off + HEADER_SIZE:]
            yield recv, label, hdr, csi


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("path", help="path to the .csi file")
    args = p.parse_args()

    if not os.path.exists(args.path):
        sys.exit(f"no such file: {args.path}")

    size = os.path.getsize(args.path)
    print(f"\nfile: {args.path}  ({size/1048576:.2f} MB)")

    per_label = {}   # label -> [count, t_first, t_last, zero_frames, sum_amp]
    widths = {}
    seqs = []
    ts_us = []
    rssi = []
    first_word_invalid = 0
    rx_state_nonzero = 0
    total = 0

    # Group by contiguous BLOCK, not by label name. The same label recurs (the
    # protocol interleaves), and merging repeats makes a block's "duration" span
    # everything in between, which reports a meaninglessly low rate.
    blocks = []       # [label, t_first, t_last, count, zero, sum_amp]
    prev_label = None

    for recv, label, hdr, csi in read_records(args.path):
        total += 1
        if label != prev_label:
            blocks.append([label, recv, recv, 0, 0, 0.0])
            prev_label = label
        b = blocks[-1]
        b[2] = recv
        b[3] += 1
        e = per_label.setdefault(label, [0, recv, recv, 0, 0.0])
        e[0] += 1
        e[2] = recv

        # All-zero I/Q means the radio handed us an empty frame: it would look
        # like a valid record but carries no channel information at all.
        if not any(csi):
            e[3] += 1
            b[4] += 1
        else:
            amp = sum(abs(v - 256 if v > 127 else v) for v in csi) / len(csi)
            e[4] += amp
            b[5] += amp

        widths[hdr["num_subcarriers"]] = widths.get(hdr["num_subcarriers"], 0) + 1
        seqs.append(hdr["sequence"])
        ts_us.append(hdr["timestamp_us"])
        rssi.append(hdr["rssi"])
        first_word_invalid += hdr["first_word_invalid"]
        if hdr["rx_state"]:
            rx_state_nonzero += 1

    if not total:
        sys.exit("  NO RECORDS -- the capture is empty.")

    print(f"records: {total}")

    # Sequence gaps = frames lost in flight or dropped on the node.
    gaps = sum(b - a - 1 for a, b in zip(seqs, seqs[1:]) if b > a + 1)
    print(f"lost (sequence gaps): {gaps}"
          f"{'  <-- data has holes' if gaps else '  (clean)'}")

    # Device-clock rate is the one analysis should use; host arrival is jittered.
    if len(ts_us) > 1:
        span = (ts_us[-1] - ts_us[0]) / 1e6
        if span > 0:
            print(f"device-clock rate: {len(ts_us)/span:.1f} Hz over {span:.1f}s")
        dts = [(b - a) / 1000.0 for a, b in zip(ts_us, ts_us[1:]) if b > a]
        if dts:
            dts_sorted = sorted(dts)
            med = dts_sorted[len(dts_sorted) // 2]
            p95 = dts_sorted[int(len(dts_sorted) * 0.95)]
            print(f"inter-frame gap: median {med:.1f} ms, p95 {p95:.1f} ms, "
                  f"max {max(dts):.0f} ms")

    print(f"rssi: min {min(rssi)} / max {max(rssi)} dBm")
    print(f"frame widths: {dict(sorted(widths.items()))}"
          f"{'  <-- MIXED: group by width in analysis' if len(widths) > 1 else ''}")
    if first_word_invalid:
        print(f"first_word_invalid set on {first_word_invalid} frames "
              f"(skip word 0 on those)")
    if rx_state_nonzero:
        print(f"rx_state nonzero on {rx_state_nonzero} frames (driver flagged them)")

    print(f"\nper BLOCK (contiguous run of one label):")
    print(f"{'#':>3} {'label':<28} {'frames':>7} {'sec':>6} {'Hz':>6} {'zero':>5} {'mean|iq|':>9}")
    print("-" * 70)
    rates = {}
    for i, (label, t0, t1, n, zeros, amp) in enumerate(blocks, 1):
        dur = t1 - t0
        hz = n / dur if dur > 0 else 0
        mean_amp = amp / max(n - zeros, 1)
        print(f"{i:>3} {label:<28} {n:>7} {dur:>6.1f} {hz:>6.1f} {zeros:>5} {mean_amp:>9.1f}")
        if not label.startswith("transition_") and label != "unlabeled":
            rates.setdefault(label, []).append(hz)
    # Compare conditions on their per-block rates.
    rates = {k: sum(v) / len(v) for k, v in rates.items()}

    # The headline confound: if frame rate tracks the condition, a "signal" may
    # just be a traffic difference. Flag it here rather than after analysis.
    print()
    if len(rates) > 1:
        lo, hi = min(rates.values()), max(rates.values())
        if lo > 0 and hi / lo > 1.5:
            worst_lo = min(rates, key=rates.get)
            worst_hi = max(rates, key=rates.get)
            print(f"WARNING: frame rate varies {lo:.1f}-{hi:.1f} Hz across conditions "
                  f"({worst_lo} vs {worst_hi}).")
            print("  A rate difference can masquerade as a presence signal. Prefer")
            print("  comparisons between blocks with similar rates, and consider")
            print("  generating steady traffic (ping -t) during capture.")
        else:
            print(f"frame rate consistent across conditions ({lo:.1f}-{hi:.1f} Hz). Good.")

    zero_total = sum(v[3] for v in per_label.values())
    if zero_total:
        print(f"WARNING: {zero_total} frames have all-zero I/Q ({zero_total/total:.1%}).")

    # Nyquist: what can this capture actually resolve?
    if len(ts_us) > 1 and span > 0:
        fs = len(ts_us) / span
        print(f"\nNyquist limit {fs/2:.1f} Hz -- breathing (0.1-0.5 Hz) and "
              f"heart rate (0.8-2.0 Hz) are {'well within range' if fs/2 > 4 else 'MARGINAL'}.")

    meta_path = args.path.replace(".csi", ".json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        if meta.get("frames") != total:
            print(f"\nnote: sidecar says {meta.get('frames')} frames, file has {total} "
                  f"(differs if the run was killed mid-write)")


if __name__ == "__main__":
    main()
