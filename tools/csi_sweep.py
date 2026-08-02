"""
Exhaustive feature sweep for CSI presence detection.

Scores EVERY feature in csi_features.py against every condition, and — the part
that matters — scores each one against an empty-vs-empty control so a feature
that "detects" people in an empty room is exposed rather than believed.

Why this exists: an earlier hand-picked analysis used only fluctuation features
and reported no signal, while a hand on the board was collapsing subcarriers to
~10% amplitude. Choosing features by intuition is how that happens. This script
chooses nothing; it ranks everything by measured performance net of its control.

Key methodology
---------------
* Segments (default 10 s) are the analysis unit; whole blocks are too few.
* Permutation is at BLOCK level, because segments within a block are dependent.
* Every feature reports (separability, control separability, margin). Only
  margin -- real minus control -- is evidence.
* Per-subcarrier features are scored two ways: best single subcarrier (with a
  multiple-comparison penalty noted) and band-aggregated.
* Reference-profile features (distance from the empty-room profile) are computed
  with a leave-one-block-out reference so a segment is never compared against a
  reference built from itself.

Usage:
    python csi_sweep.py data/session1.csi
    python csi_sweep.py data/session1.csi --seg 20 --top 25
    python csi_sweep.py data/session1.csi --condition next_room
"""

import argparse
import itertools
import math
import os
import struct
import sys

import csi_features as F

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

PRESENT = ("hand_on_board", "sitting", "moving", "next_room")
ABSENT = ("empty", "empty_end")


# --------------------------------------------------------------- data load

def list_nodes(path):
    """Node IDs present in a capture, with frame counts."""
    seen = {}
    with open(path, "rb") as f:
        while True:
            raw = f.read(4)
            if len(raw) < 4:
                break
            (n,) = struct.unpack("<I", raw)
            rec = f.read(n)
            if len(rec) < n:
                break
            _recv, ll = struct.unpack_from("<dB", rec, 0)
            hdr = struct.unpack_from(HEADER_FMT, rec, 9 + ll)
            seen[hdr[1]] = seen.get(hdr[1], 0) + 1
    return seen


def node_widths(path):
    """Per-node subcarrier-count distribution.

    Chip families differ here: the original ESP32 and the C3 do not necessarily
    report the same CSI layout for the same frame type. Features are only
    comparable within one width, so this has to be checked before analysis
    rather than assumed.
    """
    out = {}
    with open(path, "rb") as f:
        while True:
            raw = f.read(4)
            if len(raw) < 4:
                break
            (n,) = struct.unpack("<I", raw)
            rec = f.read(n)
            if len(rec) < n:
                break
            _recv, ll = struct.unpack_from("<dB", rec, 0)
            h = struct.unpack_from(HEADER_FMT, rec, 9 + ll)
            out.setdefault(h[1], {})
            out[h[1]][h[3]] = out[h[1]].get(h[3], 0) + 1
    return out


def load_segments(path, seg_sec, width=64, node=None):
    """Read the capture into contiguous blocks, then split into segments.

    `node` selects one node's stream. With several nodes streaming into one file
    their frames interleave, and treating that as a single time series would be
    wrong: consecutive records would alternate between two different physical
    vantage points. Each node must be analysed as its own series.
    """
    blocks, prev, skipped = [], None, {"width": 0, "state": 0, "node": 0}
    with open(path, "rb") as f:
        while True:
            raw = f.read(4)
            if len(raw) < 4:
                break
            (n,) = struct.unpack("<I", raw)
            rec = f.read(n)
            if len(rec) < n:
                break
            _recv, ll = struct.unpack_from("<dB", rec, 0)
            off = 9
            label = rec[off:off + ll].decode("utf-8", "replace")
            off += ll
            hdr = dict(zip(HEADER_FIELDS, struct.unpack_from(HEADER_FMT, rec, off)))
            csi = rec[off + HEADER_SIZE:]

            if node is not None and hdr["node_id"] != node:
                skipped["node"] += 1
                continue
            if hdr["num_subcarriers"] != width:
                skipped["width"] += 1
                continue
            if hdr["rx_state"]:
                skipped["state"] += 1
                continue

            start = 2 if hdr["first_word_invalid"] else 0
            amps = []
            for i in range(start, len(csi) // 2):
                a, b = csi[2 * i], csi[2 * i + 1]
                im = a - 256 if a > 127 else a
                re = b - 256 if b > 127 else b
                amps.append(math.hypot(re, im))

            if label != prev:
                blocks.append({"label": label, "ts": [], "amps": [], "rssi": []})
                prev = label
            blocks[-1]["ts"].append(hdr["timestamp_us"])
            blocks[-1]["amps"].append(amps)
            blocks[-1]["rssi"].append(hdr["rssi"])

    real = [b for b in blocks
            if not b["label"].startswith("transition_") and b["label"] != "unlabeled"]

    segs = []
    for bi, b in enumerate(real):
        cur_t, cur_a, cur_r, t0 = [], [], [], b["ts"][0]
        for t, a, r in zip(b["ts"], b["amps"], b["rssi"]):
            if (t - t0) / 1e6 >= seg_sec:
                if len(cur_t) >= 32:
                    segs.append({"label": b["label"], "block_id": bi,
                                 "ts": cur_t, "amps": cur_a, "rssi": cur_r})
                cur_t, cur_a, cur_r, t0 = [], [], [], t
            cur_t.append(t)
            cur_a.append(a)
            cur_r.append(r)
        if len(cur_t) >= 32:
            segs.append({"label": b["label"], "block_id": bi,
                         "ts": cur_t, "amps": cur_a, "rssi": cur_r})
    return real, segs, skipped


# ------------------------------------------------------------- computation

def compute_features(segs, verbose=True):
    """Compute every feature for every segment.

    Returns per-subcarrier dict {name: [scalar per sc]} and scalar dict per seg.
    """
    n_seg = len(segs)
    for i, seg in enumerate(segs):
        if verbose:
            sys.stdout.write(f"\r  computing features: segment {i+1}/{n_seg}   ")
            sys.stdout.flush()
        n_sc = min(len(a) for a in seg["amps"])
        sc_series = [[a[sc] for a in seg["amps"]] for sc in range(n_sc)]
        ts = seg["ts"]

        per_sc = {name: [] for name in F.PER_SC}
        for sc in range(n_sc):
            cache = {}
            s = sc_series[sc]
            for name, fn in F.PER_SC.items():
                try:
                    per_sc[name].append(fn(s, ts, cache))
                except (ValueError, ZeroDivisionError, OverflowError):
                    per_sc[name].append(0.0)

        scal = {}
        for name, fn in F.SEGMENT.items():
            try:
                scal[name] = fn(seg, sc_series, ts, {})
            except (ValueError, ZeroDivisionError, OverflowError):
                scal[name] = 0.0

        seg["per_sc"] = per_sc
        seg["scalar"] = scal
        seg["n_sc"] = n_sc
    if verbose:
        sys.stdout.write("\r" + " " * 60 + "\r")
    return segs


def add_reference_features(segs, n_sc):
    """Distance-from-empty-profile features, leave-one-block-out.

    A segment must never be compared against a reference that includes its own
    block, or it would be measuring itself and look artificially good.
    """
    absent = [s for s in segs if s["label"] in ABSENT]
    abs_blocks = sorted({s["block_id"] for s in absent})

    for s in segs:
        pool = [o for o in absent if o["block_id"] != s["block_id"]]
        if not pool:
            pool = absent
        for base in ("mean", "var", "mad", "cv"):
            ref = [F.mean([o["per_sc"][base][sc] for o in pool]) for sc in range(n_sc)]
            v = s["per_sc"][base]
            # L1 (robust) and L2 (emphasises large single-subcarrier changes),
            # plus cosine distance which ignores overall scale and sees only
            # SHAPE -- useful because AGC can rescale everything uniformly.
            s["scalar"][f"dist_L1_{base}"] = F.mean(
                [abs(v[sc] - ref[sc]) for sc in range(n_sc)])
            s["scalar"][f"dist_L2_{base}"] = math.sqrt(
                sum((v[sc] - ref[sc]) ** 2 for sc in range(n_sc)) / n_sc)
            num = sum(v[sc] * ref[sc] for sc in range(n_sc))
            dv = math.sqrt(sum(x * x for x in v))
            dr = math.sqrt(sum(x * x for x in ref))
            s["scalar"][f"dist_cos_{base}"] = (1 - num / (dv * dr)) if dv and dr else 0.0
            # Correlation with the reference profile: 1.0 means identical shape.
            s["scalar"][f"corr_ref_{base}"] = F.pearson(v, ref)
            # Max single-subcarrier deviation: a body may hit only a few.
            s["scalar"][f"dist_max_{base}"] = max(
                abs(v[sc] - ref[sc]) for sc in range(n_sc))
            # How many subcarriers deviate strongly (relative to ref spread).
            spread = F.stdev(ref) or 1.0
            s["scalar"][f"n_dev_{base}"] = sum(
                1 for sc in range(n_sc) if abs(v[sc] - ref[sc]) > 2 * spread)
    return segs


# -------------------------------------------------------------- statistics

def auc(pos, neg):
    if not pos or not neg:
        return 0.5
    merged = sorted([(v, 0) for v in neg] + [(v, 1) for v in pos])
    ranks, i = {}, 0
    while i < len(merged):
        j = i
        while j + 1 < len(merged) and merged[j + 1][0] == merged[i][0]:
            j += 1
        r = 0.5 * (i + j) + 1
        for k in range(i, j + 1):
            ranks[k] = r
        i = j + 1
    rs = sum(ranks[k] for k, (_v, g) in enumerate(merged) if g == 1)
    n1, n0 = len(pos), len(neg)
    return (rs - n1 * (n1 + 1) / 2) / (n1 * n0)


def sep(pos, neg):
    """Separability: |AUC-0.5|*2, so 0 = useless and 1 = perfect either way."""
    return abs(auc(pos, neg) - 0.5) * 2


def cohens_d(pos, neg):
    if len(pos) < 2 or len(neg) < 2:
        return 0.0
    vp, vn = F.variance(pos), F.variance(neg)
    n1, n2 = len(pos), len(neg)
    pooled = math.sqrt(((n1 - 1) * vp + (n2 - 1) * vn) / (n1 + n2 - 2))
    return (F.mean(pos) - F.mean(neg)) / pooled if pooled > 0 else 0.0


def block_permutation(values, seg_blocks, block_is_present, max_perm=5000):
    """p-value with whole BLOCKS permuted (segments are not independent)."""
    blocks = sorted(set(seg_blocks))
    n_pres = sum(1 for b in blocks if block_is_present[b])
    if n_pres == 0 or n_pres == len(blocks):
        return 1.0

    def stat(assign):
        p = [v for v, b in zip(values, seg_blocks) if assign[b]]
        q = [v for v, b in zip(values, seg_blocks) if not assign[b]]
        if not p or not q:
            return 0.0
        return abs(F.mean(p) - F.mean(q))

    observed = stat(block_is_present)
    combos = list(itertools.combinations(blocks, n_pres))
    if len(combos) > max_perm:
        step = max(1, len(combos) // max_perm)
        combos = combos[::step][:max_perm]
    hits = 0
    for c in combos:
        s = set(c)
        if stat({b: (b in s) for b in blocks}) >= observed - 1e-12:
            hits += 1
    return hits / len(combos)


def control_sep(feature_values, absent_segs, get):
    """Separability between empty blocks — the false-positive floor.

    Uses the best pairing of empty blocks, i.e. the WORST case for us, so a
    feature must beat the most drift-prone comparison to count as evidence.
    """
    blocks = sorted({s["block_id"] for s in absent_segs})
    if len(blocks) < 2:
        return 0.0
    worst = 0.0
    for a, b in itertools.combinations(blocks, 2):
        pa = [get(s) for s in absent_segs if s["block_id"] == a]
        pb = [get(s) for s in absent_segs if s["block_id"] == b]
        if pa and pb:
            worst = max(worst, sep(pa, pb))
    return worst


# ------------------------------------------------------------------ report

def evaluate(segs, n_sc, conditions, top_n, do_perm):
    absent = [s for s in segs if s["label"] in ABSENT]
    results = []

    for cond in conditions:
        cs = [s for s in segs if s["label"] == cond]
        if not cs:
            continue

        # --- scalar features
        for name in sorted(segs[0]["scalar"]):
            get = lambda s, n=name: s["scalar"][n]
            pos = [get(s) for s in cs]
            neg = [get(s) for s in absent]
            r = sep(pos, neg)
            ctrl = control_sep(None, absent, get)
            results.append({
                "cond": cond, "feature": name, "kind": "scalar", "sc": None,
                "sep": r, "ctrl": ctrl, "margin": r - ctrl,
                "d": cohens_d(pos, neg),
                "mean_pos": F.mean(pos), "mean_neg": F.mean(neg),
            })

        # --- per-subcarrier: best single sc, and band mean
        for name in sorted(F.PER_SC):
            best = None
            for sc in range(n_sc):
                get = lambda s, n=name, k=sc: s["per_sc"][n][k]
                pos = [get(s) for s in cs]
                neg = [get(s) for s in absent]
                r = sep(pos, neg)
                if best is None or r > best[0]:
                    best = (r, sc, get, pos, neg)
            r, sc, get, pos, neg = best
            ctrl = control_sep(None, absent, get)
            results.append({
                "cond": cond, "feature": name, "kind": "per_sc(best)", "sc": sc,
                "sep": r, "ctrl": ctrl, "margin": r - ctrl,
                "d": cohens_d(pos, neg),
                "mean_pos": F.mean(pos), "mean_neg": F.mean(neg),
            })

            getb = lambda s, n=name: F.mean(s["per_sc"][n])
            pos = [getb(s) for s in cs]
            neg = [getb(s) for s in absent]
            r = sep(pos, neg)
            ctrl = control_sep(None, absent, getb)
            results.append({
                "cond": cond, "feature": name, "kind": "per_sc(bandavg)", "sc": None,
                "sep": r, "ctrl": ctrl, "margin": r - ctrl,
                "d": cohens_d(pos, neg),
                "mean_pos": F.mean(pos), "mean_neg": F.mean(neg),
            })

    # Optional permutation p-values for the strongest survivors only: the test
    # is expensive and meaningless for features that already fail the control.
    if do_perm:
        block_present = {}
        for s in segs:
            block_present[s["block_id"]] = s["label"] in PRESENT
        strong = sorted([r for r in results if r["margin"] > 0.2],
                        key=lambda r: -r["margin"])[:40]
        for r in strong:
            cs = [s for s in segs if s["label"] == r["cond"]]
            used = cs + absent
            if r["kind"] == "scalar":
                get = lambda s, n=r["feature"]: s["scalar"][n]
            elif r["sc"] is not None:
                get = lambda s, n=r["feature"], k=r["sc"]: s["per_sc"][n][k]
            else:
                get = lambda s, n=r["feature"]: F.mean(s["per_sc"][n])
            vals = [get(s) for s in used]
            blks = [s["block_id"] for s in used]
            r["p"] = block_permutation(vals, blks, block_present)

    return results


def print_report(real, segs, skipped, results, n_sc, conditions, top_n):
    print(f"\n{'='*88}")
    print("  EXHAUSTIVE CSI FEATURE SWEEP")
    print(f"{'='*88}")

    print(f"\nblocks: {len(real)}   segments: {len(segs)}   subcarriers: {n_sc}")
    if skipped["width"]:
        print(f"skipped {skipped['width']} frames of mismatched width")
    if skipped["state"]:
        print(f"skipped {skipped['state']} frames with rx_state set")
    n_feat = len(F.PER_SC) * 2 + len(segs[0]["scalar"])
    print(f"features evaluated: {len(F.PER_SC)} per-subcarrier (x best+bandavg) "
          f"+ {len(segs[0]['scalar'])} scalar = {n_feat} per condition")

    counts = {}
    for s in segs:
        counts[s["label"]] = counts.get(s["label"], 0) + 1
    print("\nsegments per label: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))

    print(f"\n{'-'*88}")
    print("HOW TO READ THIS")
    print(f"{'-'*88}")
    print("  sep    = separability vs empty  (0 = useless, 1 = perfect)")
    print("  ctrl   = same feature on empty-vs-empty  (its FALSE-POSITIVE floor)")
    print("  margin = sep - ctrl  <-- THE ONLY COLUMN THAT IS EVIDENCE")
    print("  A feature with sep 1.00 and ctrl 1.00 detects nothing; it just drifts.")

    for cond in conditions:
        rs = [r for r in results if r["cond"] == cond]
        if not rs:
            continue
        rs.sort(key=lambda r: (-r["margin"], -r["sep"]))
        print(f"\n{'='*88}")
        print(f"  CONDITION: {cond}   (top {top_n} by margin)")
        print(f"{'='*88}")
        print(f"  {'feature':<22} {'kind':<16} {'sc':>3} {'sep':>5} {'ctrl':>5} "
              f"{'margin':>7} {'d':>6} {'p':>7}")
        print("  " + "-" * 84)
        for r in rs[:top_n]:
            sc = "" if r["sc"] is None else str(r["sc"])
            p = f"{r['p']:.4f}" if "p" in r else ""
            print(f"  {r['feature']:<22} {r['kind']:<16} {sc:>3} {r['sep']:>5.2f} "
                  f"{r['ctrl']:>5.2f} {r['margin']:>+7.2f} {r['d']:>+6.2f} {p:>7}")

        clean = [r for r in rs if r["margin"] > 0.3 and r["sep"] > 0.7]
        if clean:
            print(f"\n  {len(clean)} features clear margin>0.3 AND sep>0.7:")
            for r in clean[:8]:
                print(f"    {r['feature']} ({r['kind']}"
                      f"{'' if r['sc'] is None else ' sc'+str(r['sc'])}): "
                      f"{r['mean_neg']:.4g} -> {r['mean_pos']:.4g}")
        else:
            print(f"\n  NO feature clears margin>0.3 with sep>0.7. "
                  f"No reliable detection for '{cond}' in this data.")

    # Cross-condition summary: the honest headline.
    print(f"\n{'='*88}")
    print("  SUMMARY: best margin per condition")
    print(f"{'='*88}")
    print(f"  {'condition':<16} {'best margin':>12} {'feature':<24} {'kind':<16} {'verdict'}")
    for cond in conditions:
        rs = [r for r in results if r["cond"] == cond]
        if not rs:
            continue
        b = max(rs, key=lambda r: r["margin"])
        if b["margin"] > 0.5:
            v = "STRONG"
        elif b["margin"] > 0.3:
            v = "moderate"
        elif b["margin"] > 0.15:
            v = "weak / suggestive"
        else:
            v = "NOT DETECTED"
        print(f"  {cond:<16} {b['margin']:>+12.2f} {b['feature']:<24} "
              f"{b['kind']:<16} {v}")

    # Confound check: does frame rate alone separate the classes?
    print(f"\n{'-'*88}")
    print("CONFOUND CHECK (frame_rate / rssi separating classes is a WARNING, "
          "not a win)")
    print(f"{'-'*88}")
    for cond in conditions:
        for fname in ("frame_rate", "rssi_mean", "iat_std"):
            hit = [r for r in results
                   if r["cond"] == cond and r["feature"] == fname and r["kind"] == "scalar"]
            if hit and hit[0]["margin"] > 0.3:
                print(f"  {cond}: '{fname}' margin {hit[0]['margin']:+.2f} -- "
                      f"a link-layer effect may be driving apparent detection")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--seg", type=float, default=10.0, help="segment seconds")
    ap.add_argument("--top", type=int, default=20, help="rows per condition")
    ap.add_argument("--condition", action="append", help="restrict to condition(s)")
    ap.add_argument("--no-perm", action="store_true", help="skip permutation tests")
    ap.add_argument("--node", type=int, help="analyse only this node_id")
    ap.add_argument("--width", type=int, default=64,
                    help="subcarrier count to keep (chip families differ)")
    a = ap.parse_args()

    if not os.path.exists(a.path):
        sys.exit(f"no such file: {a.path}")

    nodes = list_nodes(a.path)
    if len(nodes) > 1:
        print(f"capture contains {len(nodes)} nodes: " +
              ", ".join(f"node {k}={v} frames" for k, v in sorted(nodes.items())))
        nw = node_widths(a.path)
        print("dominant CSI width per node:")
        for k in sorted(nw):
            dom = max(nw[k].items(), key=lambda kv: kv[1])
            print(f"  node {k}: {dict(sorted(nw[k].items()))}  -> use --width {dom[0]}")
        widths = {max(v.items(), key=lambda kv: kv[1])[0] for v in nw.values()}
        if len(widths) > 1:
            print("  NOTE: nodes report DIFFERENT widths (chip families differ).")
            print("        Per-subcarrier features are only comparable within a "
                  "width, so\n        compare nodes by RANK/direction, not by raw "
                  "feature values.")
        if a.node is None:
            print("  --> analyse each node separately; re-run with --node N "
                  "(and matching --width).\n      (their frames interleave, so a "
                  "combined series would mix vantage points)")
            for k in sorted(nw):
                dom = max(nw[k].items(), key=lambda kv: kv[1])[0]
                print(f"      python {os.path.basename(__file__)} {a.path} "
                      f"--node {k} --width {dom}")
            sys.exit(1)

    print(f"loading {a.path} (segments of {a.seg:.0f}s"
          f"{f', node {a.node}' if a.node is not None else ''})...")
    real, segs, skipped = load_segments(a.path, a.seg, width=a.width, node=a.node)
    if not segs:
        sys.exit("no usable segments")

    segs = compute_features(segs)
    n_sc = min(s["n_sc"] for s in segs)
    segs = add_reference_features(segs, n_sc)

    conditions = a.condition or [c for c in PRESENT
                                 if any(s["label"] == c for s in segs)]
    results = evaluate(segs, n_sc, conditions, a.top, not a.no_perm)
    print_report(real, segs, skipped, results, n_sc, conditions, a.top)


if __name__ == "__main__":
    main()
