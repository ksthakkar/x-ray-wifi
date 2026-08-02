"""
Offline analysis of a labeled CSI capture: does presence produce a detectable
signal, and is it distinguishable from chance?

Pure standard library (no numpy) so it runs anywhere.

Design decisions that matter for correctness:

  * Per-BLOCK, never per-label. The protocol interleaves, so the same label
    recurs; treating all `empty` frames as one pool would merge recordings
    minutes apart and hide drift.

  * Per-subcarrier, never band-averaged. A body perturbs a handful of
    subcarriers strongly; averaging over all 64 divides that by 64 and buries
    it. This is the single most important difference from the on-device score.

  * Amplitude, not phase. Raw CSI phase is dominated by per-packet timing
    offsets that we cannot correct without a reference, so phase statistics
    would mostly measure clock noise.

  * BLOCK-level permutation for significance. CSI frames at ~20 Hz are heavily
    autocorrelated: consecutive frames are near-identical, so the effective
    sample size is the number of BLOCKS (~11), not frames (~10,000). A t-test
    over frames would report p < 1e-50 for pure noise. We permute whole block
    labels instead, which respects that structure.

  * empty-vs-empty control. The three `empty` blocks contain no person, so any
    "signal" the same pipeline finds between them is the false-positive floor.
    A result only means something if it exceeds that floor.

Usage:
    python csi_analyze.py data/session1.csi
"""

import argparse
import itertools
import math
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

PRESENT = {"hand_on_board", "sitting", "moving", "next_room"}
ABSENT = {"empty", "empty_end"}


# ----------------------------------------------------------------- reading

def read_blocks(path, want_width=64):
    """Return [{label, frames:[(ts_us, [amp per sc])], ...}] as contiguous blocks.

    Frames of an unexpected width are skipped: subcarrier i means a different
    frequency in a 128-wide frame than a 64-wide one, so mixing them would
    corrupt every per-subcarrier statistic.
    """
    blocks = []
    prev = None
    skipped_width = 0
    skipped_state = 0

    with open(path, "rb") as f:
        while True:
            raw = f.read(4)
            if len(raw) < 4:
                break
            (n,) = struct.unpack("<I", raw)
            rec = f.read(n)
            if len(rec) < n:
                break
            _recv, lbl_len = struct.unpack_from("<dB", rec, 0)
            off = 9
            label = rec[off:off + lbl_len].decode("utf-8", "replace")
            off += lbl_len
            hdr = dict(zip(HEADER_FIELDS, struct.unpack_from(HEADER_FMT, rec, off)))
            csi = rec[off + HEADER_SIZE:]

            if hdr["num_subcarriers"] != want_width:
                skipped_width += 1
                continue
            if hdr["rx_state"]:
                skipped_state += 1
                continue

            # ESP-IDF orders each pair (imag, real); amplitude is symmetric in
            # the two so the ordering does not matter here.
            amps = []
            start = 2 if hdr["first_word_invalid"] else 0
            for i in range(start, len(csi) // 2):
                a = csi[2 * i]
                b = csi[2 * i + 1]
                im = a - 256 if a > 127 else a
                re = b - 256 if b > 127 else b
                amps.append(math.hypot(re, im))

            if label != prev:
                blocks.append({"label": label, "frames": [], "rssi": []})
                prev = label
            blocks[-1]["frames"].append((hdr["timestamp_us"], amps))
            blocks[-1]["rssi"].append(hdr["rssi"])

    return blocks, skipped_width, skipped_state


# ------------------------------------------------------------- statistics

def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def variance(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return sum((x - m) ** 2 for x in xs) / (len(xs) - 1)


def stdev(xs):
    return math.sqrt(variance(xs))


def median(xs):
    if not xs:
        return 0.0
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def auc(pos, neg):
    """Probability a random `pos` sample exceeds a random `neg` one.

    Rank-based (Mann-Whitney), so it needs no distributional assumption.
    0.5 = indistinguishable, 1.0 = perfectly separable. Reported as |AUC-0.5|
    doubled ("separability") elsewhere so direction does not matter.
    """
    if not pos or not neg:
        return 0.5
    merged = sorted([(v, 0) for v in neg] + [(v, 1) for v in pos])
    # Average ranks over ties, else tied values bias the estimate.
    ranks = {}
    i = 0
    while i < len(merged):
        j = i
        while j + 1 < len(merged) and merged[j + 1][0] == merged[i][0]:
            j += 1
        r = 0.5 * (i + j) + 1
        for k in range(i, j + 1):
            ranks.setdefault(k, r)
        i = j + 1
    rank_sum_pos = sum(ranks[k] for k, (_v, g) in enumerate(merged) if g == 1)
    n1, n0 = len(pos), len(neg)
    u = rank_sum_pos - n1 * (n1 + 1) / 2
    return u / (n1 * n0)


def block_feature(block, feat):
    """Collapse a block to one number per subcarrier.

    Block-level features (not per-frame) because blocks are the independent
    unit; per-frame values are autocorrelated and would inflate significance.
    """
    frames = block["frames"]
    if len(frames) < 4:
        return None
    n_sc = min(len(a) for _t, a in frames)
    out = []
    for sc in range(n_sc):
        series = [a[sc] for _t, a in frames]
        if feat == "var":
            out.append(variance(series))
        elif feat == "mean":
            out.append(mean(series))
        elif feat == "mad":
            # Mean absolute first difference: frame-to-frame agitation. Robust
            # and insensitive to slow drift, unlike raw variance.
            d = [abs(series[i + 1] - series[i]) for i in range(len(series) - 1)]
            out.append(mean(d))
        elif feat == "bandpower":
            out.append(band_power(frames, sc, 0.1, 0.6))
    return out


def profile_distance(segs, ref, n_sc):
    """L1 distance of each segment's mean-amplitude profile from a reference.

    This is the feature that actually works, and the reason is worth stating:
    var/mad/bandpower all measure how much a subcarrier FLUCTUATES. A body
    changes the channel's SHAPE -- which subcarriers are strong and which are
    nulled -- and that shows up in the mean profile, not in its agitation.
    Measuring only fluctuation misses a large, static, obvious perturbation.
    """
    return [mean([abs(s["mean"][sc] - ref[sc]) for sc in range(n_sc)]) for s in segs]


def band_power(frames, sc, f_lo, f_hi):
    """Power in [f_lo, f_hi] Hz for one subcarrier, via a direct DFT.

    Uses real timestamps (not a nominal rate) because the CSI rate is variable;
    resamples onto a uniform grid first since a DFT assumes even spacing.
    Detrends to stop slow drift leaking into the low-frequency bins.
    """
    ts = [t for t, _a in frames]
    vals = [a[sc] for _t, a in frames]
    span = (ts[-1] - ts[0]) / 1e6
    if span < 4.0 or len(vals) < 32:
        return 0.0

    # Uniform resample at the median rate.
    fs = len(vals) / span
    n = int(span * fs)
    if n < 32:
        return 0.0
    grid = []
    j = 0
    for k in range(n):
        target = ts[0] + k / fs * 1e6
        while j + 1 < len(ts) - 1 and ts[j + 1] < target:
            j += 1
        grid.append(vals[j])

    # Remove mean and linear trend.
    m = mean(grid)
    g = [v - m for v in grid]
    xm = (n - 1) / 2
    sxy = sum((i - xm) * g[i] for i in range(n))
    sxx = sum((i - xm) ** 2 for i in range(n))
    slope = sxy / sxx if sxx else 0.0
    g = [g[i] - slope * (i - xm) for i in range(n)]

    # Hann window to limit spectral leakage.
    g = [g[i] * 0.5 * (1 - math.cos(2 * math.pi * i / (n - 1))) for i in range(n)]

    k_lo = max(1, int(f_lo * n / fs))
    k_hi = min(n // 2, int(f_hi * n / fs) + 1)
    total = 0.0
    for k in range(k_lo, k_hi):
        re = im = 0.0
        w = 2 * math.pi * k / n
        for i in range(n):
            re += g[i] * math.cos(w * i)
            im -= g[i] * math.sin(w * i)
        total += (re * re + im * im) / (n * n)
    return total


# ----------------------------------------------------------- permutation

def permutation_test(labels, values, n_present, max_perm=20000):
    """P-value for the observed present/absent gap, permuting BLOCK labels.

    Exhaustive when the number of arrangements is small (it usually is with
    ~11 blocks), else randomly sampled with a fixed seed for reproducibility.
    """
    idx = list(range(len(labels)))
    obs_pos = [values[i] for i in idx if labels[i] == 1]
    obs_neg = [values[i] for i in idx if labels[i] == 0]
    if not obs_pos or not obs_neg:
        return 1.0, 0.0
    observed = abs(mean(obs_pos) - mean(obs_neg))

    combos = list(itertools.combinations(idx, n_present))
    if len(combos) > max_perm:
        # Deterministic subsample: stride through the list.
        step = len(combos) // max_perm
        combos = combos[::step][:max_perm]

    count = 0
    for c in combos:
        s = set(c)
        p = [values[i] for i in idx if i in s]
        q = [values[i] for i in idx if i not in s]
        if abs(mean(p) - mean(q)) >= observed - 1e-12:
            count += 1
    return count / len(combos), observed


# ---------------------------------------------------------------- report

def segment_blocks(blocks, seg_sec=10.0):
    """Split each block into fixed-duration segments.

    Whole blocks are too few (9) for a permutation test or an honest control:
    with 3 absent blocks, a perfect AUC arises by chance constantly. Segments
    of ~10 s are still long enough to resolve the 0.1-0.6 Hz breathing band
    (>= 1 full cycle) while giving ~50 independent-ish units instead of 9.

    Segments inherit their parent's label, and -- critically -- keep a
    `block_id` so the permutation test can shuffle whole blocks and never split
    a block across the two classes (which would leak information).
    """
    segs = []
    for bi, b in enumerate(blocks):
        fr = b["frames"]
        if not fr:
            continue
        t0 = fr[0][0]
        cur, cur_start = [], t0
        for f in fr:
            if (f[0] - cur_start) / 1e6 >= seg_sec:
                if len(cur) >= 32:
                    segs.append({"label": b["label"], "block_id": bi,
                                 "frames": cur, "rssi": []})
                cur, cur_start = [], f[0]
            cur.append(f)
        if len(cur) >= 32:
            segs.append({"label": b["label"], "block_id": bi,
                         "frames": cur, "rssi": []})
    return segs


def grouped_permutation(seg_labels, seg_blocks, values, max_perm=20000):
    """Permutation p-value that shuffles labels at BLOCK level, not segment.

    Segments within a block are highly dependent, so permuting segments would
    overstate significance exactly like permuting frames does. Instead we
    reassign whole blocks and recompute the segment-level mean difference.
    """
    blocks = sorted(set(seg_blocks))
    block_label = {}
    for b, l in zip(seg_blocks, seg_labels):
        block_label[b] = l
    n_pres_blocks = sum(1 for b in blocks if block_label[b] == 1)

    def stat(assign):
        pos = [v for v, b in zip(values, seg_blocks) if assign[b] == 1]
        neg = [v for v, b in zip(values, seg_blocks) if assign[b] == 0]
        if not pos or not neg:
            return 0.0
        return abs(mean(pos) - mean(neg))

    observed = stat(block_label)
    combos = list(itertools.combinations(blocks, n_pres_blocks))
    if len(combos) > max_perm:
        step = max(1, len(combos) // max_perm)
        combos = combos[::step][:max_perm]

    count = 0
    for c in combos:
        s = set(c)
        assign = {b: (1 if b in s else 0) for b in blocks}
        if stat(assign) >= observed - 1e-12:
            count += 1
    return count / len(combos), observed, len(combos)


def analyze(path):
    print(f"\n{'='*74}")
    print(f"  CSI PRESENCE ANALYSIS  --  {os.path.basename(path)}")
    print(f"{'='*74}")

    blocks, skip_w, skip_s = read_blocks(path)
    real = [b for b in blocks
            if not b["label"].startswith("transition_") and b["label"] != "unlabeled"]

    print(f"\nblocks: {len(blocks)} total, {len(real)} experimental "
          f"(transitions/unlabeled excluded)")
    if skip_w:
        print(f"skipped {skip_w} frames of non-64sc width (cannot be compared)")
    if skip_s:
        print(f"skipped {skip_s} frames with rx_state set (driver flagged invalid)")

    print(f"\n{'#':>3} {'label':<16} {'frames':>7} {'sec':>6} {'Hz':>5} "
          f"{'rssi':>6} {'meanamp':>8}")
    print("-" * 60)
    for i, b in enumerate(real, 1):
        fr = b["frames"]
        span = (fr[-1][0] - fr[0][0]) / 1e6
        allamp = [v for _t, a in fr for v in a]
        print(f"{i:>3} {b['label']:<16} {len(fr):>7} {span:>6.1f} "
              f"{len(fr)/span if span else 0:>5.1f} {mean(b['rssi']):>6.1f} "
              f"{mean(allamp):>8.2f}")

    # ---- 1. Whole-band summary: what the on-device score effectively saw.
    print(f"\n{'-'*74}")
    print("1. WHOLE-BAND (what the firmware's averaged score effectively measured)")
    print(f"{'-'*74}")
    print(f"{'label':<16} {'mean|H|':>9} {'band-avg var':>13} {'band-avg MAD':>13}")
    for b in real:
        v = block_feature(b, "var")
        m = block_feature(b, "mad")
        allamp = [x for _t, a in b["frames"] for x in a]
        print(f"{b['label']:<16} {mean(allamp):>9.3f} {mean(v):>13.4f} {mean(m):>13.4f}")

    # ---- 2. Per-subcarrier: does any single subcarrier separate the classes?
    print(f"\n{'-'*74}")
    print("2. PER-SUBCARRIER SEPARABILITY (present vs absent, per block)")
    print(f"{'-'*74}")

    # "mean" is included deliberately: the first version of this script used only
    # fluctuation features (var/mad/bandpower) and concluded there was no signal,
    # when a hand on the board was in fact collapsing whole subcarriers to ~10%
    # of their empty-room amplitude. Shape change != agitation.
    feats = ["mean", "var", "mad", "bandpower"]

    # Segment into ~10 s units. Whole blocks (9) are too few for the control to
    # be meaningful: with 3 absent blocks a perfect AUC occurs by chance.
    segs = [s for s in segment_blocks(real, 10.0)]
    cache = {}
    for s in segs:
        for f in feats:
            cache[(id(s), f)] = block_feature(s, f)
    segs = [s for s in segs if cache[(id(s), "var")] is not None]

    pres_segs = [s for s in segs if s["label"] in PRESENT]
    abs_segs = [s for s in segs if s["label"] in ABSENT]
    print(f"segments (10 s): {len(segs)} total -- "
          f"{len(pres_segs)} present, {len(abs_segs)} absent")

    n_sc = min(len(cache[(id(s), 'var')]) for s in segs)

    results = {}
    for f in feats:
        rows = []
        for sc in range(n_sc):
            pos = [cache[(id(s), f)][sc] for s in pres_segs]
            neg = [cache[(id(s), f)][sc] for s in abs_segs]
            a = auc(pos, neg)
            rows.append((abs(a - 0.5) * 2, a, sc, mean(pos), mean(neg)))
        rows.sort(reverse=True)
        results[f] = rows
        print(f"\n  feature '{f}' -- top 8 subcarriers by separability:")
        print(f"    {'sc':>4} {'sep':>6} {'AUC':>6} {'present':>12} {'absent':>12} {'ratio':>7}")
        for sep, a, sc, mp, mn in rows[:8]:
            ratio = mp / mn if mn else float("inf")
            print(f"    {sc:>4} {sep:>6.2f} {a:>6.2f} {mp:>12.4f} {mn:>12.4f} {ratio:>7.2f}")

    # ---- 3. Significance: permute at BLOCK level over segment features.
    print(f"\n{'-'*74}")
    print("3. SIGNIFICANCE (labels permuted per BLOCK; segments are not independent)")
    print(f"{'-'*74}")
    used = [s for s in segs if s["label"] in PRESENT or s["label"] in ABSENT]
    sl = [1 if s["label"] in PRESENT else 0 for s in used]
    sb = [s["block_id"] for s in used]
    n_blocks = len(set(sb))
    print(f"  {len(used)} segments across {n_blocks} blocks")
    print(f"  -> permutation unit is the BLOCK ({n_blocks}), not the segment or frame.")
    print(f"  {'feature':<12} {'best sc':>8} {'sep':>6} {'p':>8} {'p x 64':>8}  verdict")
    for f in feats:
        best = results[f][0]
        sc = best[2]
        vals = [cache[(id(s), f)][sc] for s in used]
        p, obs, nperm = grouped_permutation(sl, sb, vals)
        p_adj = min(1.0, p * n_sc)
        verdict = "SIGNIFICANT" if p_adj < 0.05 else ("marginal" if p_adj < 0.2
                                                     else "not significant")
        print(f"  {f:<12} {sc:>8} {best[0]:>6.2f} {p:>8.4f} {p_adj:>8.3f}  {verdict}"
              f"   ({nperm} perms)")

    # ---- 4. The control: same pipeline, empty vs empty.
    print(f"\n{'-'*74}")
    print("4. CONTROL: empty vs empty (no person present -- the false-positive floor)")
    print(f"{'-'*74}")
    empty_blocks = sorted({s["block_id"] for s in abs_segs})
    if len(empty_blocks) >= 2:
        # Split by BLOCK so the two halves are genuinely different recordings.
        half = len(empty_blocks) // 2
        gA = set(empty_blocks[:half])
        a_segs = [s for s in abs_segs if s["block_id"] in gA]
        b_segs = [s for s in abs_segs if s["block_id"] not in gA]
        print(f"  group A: {len(a_segs)} segs from blocks {sorted(gA)}; "
              f"group B: {len(b_segs)} segs")
        for f in feats:
            rows = []
            for sc in range(n_sc):
                pos = [cache[(id(s), f)][sc] for s in a_segs]
                neg = [cache[(id(s), f)][sc] for s in b_segs]
                rows.append((abs(auc(pos, neg) - 0.5) * 2, sc))
            rows.sort(reverse=True)
            real_best = results[f][0][0]
            if rows[0][0] >= real_best - 0.02:
                flag = "  <-- CONTROL >= REAL: this feature is not evidence"
            elif rows[0][0] > 0.7 * real_best:
                flag = "  <-- control is close to real: treat with caution"
            else:
                flag = "  (real result clears the floor)"
            print(f"  {f:<12} control best sep {rows[0][0]:.2f} (sc {rows[0][1]}) "
                  f"vs real {real_best:.2f}{flag}")
    else:
        print("  need >=2 empty blocks")

    # ---- 5. Per-condition, using segments so each has several units.
    print(f"\n{'-'*74}")
    print("5. EACH CONDITION vs POOLED EMPTY (which conditions are detectable?)")
    print(f"{'-'*74}")
    print(f"  {'condition':<16} {'segs':>5} {'best sep':>9} {'sc':>4} "
          f"{'ratio':>7}  {'feature':>10}")
    for cond in ["hand_on_board", "moving", "sitting", "next_room"]:
        cs = [s for s in segs if s["label"] == cond]
        if not cs:
            continue
        best = (0.0, -1, 0.0, "")
        for f in feats:
            for sc in range(n_sc):
                pos = [cache[(id(s), f)][sc] for s in cs]
                neg = [cache[(id(s), f)][sc] for s in abs_segs]
                sep = abs(auc(pos, neg) - 0.5) * 2
                if sep > best[0]:
                    r = mean(pos) / mean(neg) if mean(neg) else 0.0
                    best = (sep, sc, r, f)
        print(f"  {cond:<16} {len(cs):>5} {best[0]:>9.2f} {best[1]:>4} "
              f"{best[2]:>7.2f}  {best[3]:>10}")

    # ---- 6. Profile-distance detector: the one that actually separates.
    print(f"\n{'-'*74}")
    print("6. PROFILE-DISTANCE DETECTOR (L1 distance of mean profile from empty)")
    print(f"{'-'*74}")
    ref = [mean([cache[(id(s), "mean")][sc] for s in abs_segs]) for sc in range(n_sc)]
    for s in segs:
        s["mean"] = cache[(id(s), "mean")]
    print(f"  {'condition':<16} {'segs':>5} {'mean dist':>10} {'range':>16} {'AUC vs empty':>13}")
    abs_d = profile_distance(abs_segs, ref, n_sc)
    for cond in ["hand_on_board", "moving", "sitting", "next_room", "empty", "empty_end"]:
        cs = [s for s in segs if s["label"] == cond]
        if not cs:
            continue
        d = profile_distance(cs, ref, n_sc)
        a = auc(d, abs_d) if cond not in ABSENT else float("nan")
        astr = "     (ref)" if cond in ABSENT else f"{a:>13.3f}"
        print(f"  {cond:<16} {len(cs):>5} {mean(d):>10.3f} "
              f"{min(d):>7.2f}-{max(d):<7.2f}{astr}")

    # Control on this feature too: one empty block vs the others.
    eb = sorted({s["block_id"] for s in abs_segs})
    if len(eb) >= 2:
        gA = {eb[0]}
        dA = profile_distance([s for s in abs_segs if s["block_id"] in gA], ref, n_sc)
        dB = profile_distance([s for s in abs_segs if s["block_id"] not in gA], ref, n_sc)
        ctrl = abs(auc(dA, dB) - 0.5) * 2
        print(f"\n  CONTROL empty-vs-empty separability on this feature: {ctrl:.3f}")
        print(f"  (near 0 means the feature does NOT fire on empty rooms -- "
              f"unlike var/mad above)")

    print(f"\n{'='*74}\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    a = ap.parse_args()
    if not os.path.exists(a.path):
        sys.exit(f"no such file: {a.path}")
    analyze(a.path)
