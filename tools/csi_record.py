"""
CSI capture recorder — writes raw ESP32 CSI frames to disk for offline analysis.

Deliberately does almost nothing: receive UDP, append to a file, keep counters.
No processing, no plotting, no WebSocket. The point is to never be the reason a
frame was lost or altered.

Storage format: one .csi file per session (little-endian binary records) plus a
.json sidecar with metadata, and a .labels.csv marking which condition was
active when. Binary rather than CSV because at ~50 Hz x 64 subcarriers a CSV of
per-subcarrier I/Q is ~10x larger and much slower to write; the analysis script
reads the binary directly into numpy. Use --csv if you want a CSV too.

Each record is: [4-byte record length][46-byte header][raw CSI bytes]
The length prefix makes the file recoverable even if truncated mid-write.

Typical run -- an interleaved protocol, with labels marked live:

    python csi_record.py --out data/session1 --protocol

...which prompts you through: 60s empty, 30s walking, 60s empty, ... and writes
the exact wall-clock boundaries to the .labels.csv. Interleaving matters: a
single empty block followed by a single occupied block confounds presence with
slow drift (traffic, temperature, AP behaviour), so blocks alternate instead.

Or mark labels manually while it runs:

    python csi_record.py --out data/session1
    # then press ENTER to cycle labels, or type a label name + ENTER
"""

import argparse
import asyncio
import csv
import json
import os
import socket
import struct
import sys
import time

MAGIC = 0xC5110003
HEADER_FMT = "<IBBHIQbbBBBBBBBBBBBBBBBB6sH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 46, HEADER_SIZE

HEADER_FIELDS = [
    "magic", "node_id", "num_antennas", "num_subcarriers", "sequence",
    "timestamp_us", "rssi", "noise_floor", "channel", "secondary_channel",
    "rate", "sig_mode", "mcs", "cwb", "smoothing", "not_sounding",
    "aggregation", "stbc", "fec_coding", "sgi", "ampdu_cnt", "rx_state",
    "first_word_invalid", "phy_variant", "mac", "csi_len",
]

# --- Protocol v1 (session 1). Kept for reproducing that capture only.
#
# Two flaws found after the fact, both fixed in v2 below:
#   1. Its "empty" blocks were not empty -- the operator was behind a wall, often
#      moving. So it had no true baseline, and "is presence detectable" could not
#      be answered from it at all.
#   2. One or two blocks per condition. Blocks are the unit of statistical power,
#      so with 1-vs-3 blocks the smallest achievable permutation p-value is 0.25:
#      no effect size, however large, can reach significance.
PROTOCOL_V1 = [
    ("hand_on_board", 30, "Hold your hand right over the board."),
    ("sitting", 30, "Sit still in the room, ~1-2 m from the board."),
    ("moving", 30, "Move around the room actively."),
    ("empty", 60, "Leave the room entirely. Close the door."),
    ("moving", 30, "Come back in and move around actively."),
    ("sitting", 30, "Sit still again, same spot as before."),
    ("empty", 60, "Leave the room again."),
    ("next_room", 60, "Go to the ADJACENT room and move around."),
    ("empty_end", 60, "Leave the room. Final baseline."),
]

# --- Protocol v2: many short blocks, a genuine empty baseline, explicit locations.
#
# Design rationale, in priority order:
#
#   * MANY SHORT BLOCKS. 6 blocks per condition instead of 1-2. This is the
#     single highest-leverage change: it moves the permutation p-floor from 0.25
#     to well under 0.001, so a real effect can actually register as significant.
#
#   * A TRULY EMPTY CONDITION. Nobody in the building, or at minimum two rooms
#     away and stationary. Session 1 had none, which made every "presence" claim
#     uninterpretable. This is the reference everything else is measured against.
#
#   * LOCATIONS, NOT VAGUE LABELS. "empty" hid the fact that the operator was
#     behind a wall. Names here state where the body actually is.
#
#   * INTERLEAVED. Conditions alternate so presence is decorrelated from slow
#     drift (traffic, temperature, AP behaviour).
#
#   * STILL vs MOVING SEPARATED. Session 1 showed the top features track motion,
#     not presence; keeping these apart is the only way to tell them apart.
#
# ~30 s blocks: long enough for 3+ breathing cycles at 0.1-0.5 Hz, short enough
# to fit many of them into one session.
PROTOCOL_V2 = [
    ("truly_empty", 30, "NOBODY in the room or adjacent rooms. Go two rooms away "
                        "(or outside) and stay still. THE baseline."),
    ("in_room_still", 30, "Sit still in the middle of the room."),
    ("truly_empty", 30, "Two rooms away again, stay still."),
    ("in_room_moving", 30, "Walk around the room continuously."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_a_still", 30, "Stand just behind WALL A, STAY STILL."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_a_moving", 30, "Just behind WALL A, moving around."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("in_room_still", 30, "Sit still in the middle of the room (same spot as before)."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_b_still", 30, "Stand just behind WALL B, STAY STILL."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_b_moving", 30, "Just behind WALL B, moving around."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("in_room_moving", 30, "Walk around the room continuously."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_a_still", 30, "Just behind WALL A, still."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_b_still", 30, "Just behind WALL B, still."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("in_room_still", 30, "Sit still in the middle of the room."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_a_moving", 30, "Just behind WALL A, moving."),
    ("truly_empty", 30, "Two rooms away, still."),
    ("wall_b_moving", 30, "Just behind WALL B, moving."),
    ("truly_empty", 30, "Final baseline. Two rooms away, still."),
    ("hand_on_board", 30, "Hold your hand over NODE 1's antenna. KNOWN-POSITIVE: "
                          "if analysis misses this, the pipeline is broken and any "
                          "null result elsewhere is meaningless."),
]

# Position-tagged variant: instead of coarse zones, stand at numbered marked
# spots. This is what makes position ESTIMATION (not just detection) testable --
# each spot is a labelled ground-truth coordinate. Mark spots on the floor with
# tape and record their coordinates in the session notes.
PROTOCOL_POSITIONS = [
    ("truly_empty", 30, "Nobody present. Baseline."),
    ("pos_1_still", 30, "Stand STILL on marked spot 1."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_2_still", 30, "Stand STILL on marked spot 2."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_3_still", 30, "Stand STILL on marked spot 3."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_4_still", 30, "Stand STILL on marked spot 4."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_1_still", 30, "Spot 1 again (repeat -- tests reproducibility)."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_2_still", 30, "Spot 2 again."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_3_still", 30, "Spot 3 again."),
    ("truly_empty", 30, "Leave. Baseline."),
    ("pos_4_still", 30, "Spot 4 again."),
    ("truly_empty", 30, "Final baseline."),
]

PROTOCOLS = {
    "v1": PROTOCOL_V1,
    "v2": PROTOCOL_V2,
    "positions": PROTOCOL_POSITIONS,
}
DEFAULT_PROTOCOL = PROTOCOL_V2


class Recorder:
    """Owns the output files and counters. Kept synchronous and dumb."""

    def __init__(self, out_base: str, write_csv: bool):
        self.out_base = out_base
        os.makedirs(os.path.dirname(out_base) or ".", exist_ok=True)

        self.bin_path = out_base + ".csi"
        self.meta_path = out_base + ".json"
        self.labels_path = out_base + ".labels.csv"
        self.csv_path = out_base + ".csv" if write_csv else None

        self.bin_f = open(self.bin_path, "wb")
        self.labels_f = open(self.labels_path, "w", newline="")
        self.labels_w = csv.writer(self.labels_f)
        self.labels_w.writerow(["label", "start_unix", "end_unix", "note"])
        self.labels_f.flush()

        self.csv_f = None
        self.csv_w = None
        if self.csv_path:
            self.csv_f = open(self.csv_path, "w", newline="")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(
                ["recv_unix", "label"] + HEADER_FIELDS[1:] + ["csi_hex"]
            )

        self.label = "unlabeled"
        self.label_started = time.time()
        self.label_note = ""
        self.quiet = False  # suppress the status line during a countdown
        self.node_positions = {}  # node_id -> {xy_m, note}; set before the run

        self.frames = 0
        self.bytes_written = 0
        self.bad_magic = 0
        self.short = 0
        self.lost = 0            # inferred from sequence gaps (summed over nodes)
        self.prev_seq = None
        self.per_node = {}       # node_id -> {frames, lost, prev, first, last}
        self.per_label = {}      # label -> frame count
        self.sc_counts = {}      # subcarrier count -> frames (catches width mixing)
        self.first_recv = None
        self.last_recv = None
        self.started_unix = time.time()

    def set_label(self, label: str, note: str = ""):
        """Close the current label block and open a new one."""
        now = time.time()
        if self.frames or self.label != "unlabeled":
            self.labels_w.writerow(
                [self.label, f"{self.label_started:.6f}", f"{now:.6f}", self.label_note]
            )
            self.labels_f.flush()
        self.label = label
        self.label_note = note
        self.label_started = now

    def add(self, data: bytes):
        if len(data) < HEADER_SIZE:
            self.short += 1
            return None

        vals = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        if vals[0] != MAGIC:
            self.bad_magic += 1
            return None

        hdr = dict(zip(HEADER_FIELDS, vals))
        now = time.time()

        # Sequence gaps mean UDP loss in flight (or ESP32 queue drops). Recorded
        # rather than hidden: a gap is missing data, and analysis needs to know.
        seq = hdr["sequence"]
        # Per-node bookkeeping: with several nodes streaming to one recorder,
        # each has its own sequence counter, so a single shared prev_seq would
        # report huge phantom loss as the streams interleave.
        nid = hdr["node_id"]
        st = self.per_node.setdefault(nid, {"frames": 0, "lost": 0, "prev": None,
                                            "first": now, "last": now})
        st["frames"] += 1
        st["last"] = now
        if st["prev"] is not None and seq > st["prev"] + 1:
            st["lost"] += seq - st["prev"] - 1
        st["prev"] = seq
        self.lost = sum(v["lost"] for v in self.per_node.values())

        # Length-prefixed record, then the host receive time, then the datagram
        # verbatim. Storing the raw datagram means nothing is lost to parsing
        # choices made today.
        label_b = self.label.encode()[:255]
        rec = (
            struct.pack("<dB", now, len(label_b)) + label_b + data
        )
        self.bin_f.write(struct.pack("<I", len(rec)))
        self.bin_f.write(rec)
        self.bytes_written += 4 + len(rec)

        self.frames += 1
        self.per_label[self.label] = self.per_label.get(self.label, 0) + 1
        n_sc = hdr["num_subcarriers"]
        self.sc_counts[n_sc] = self.sc_counts.get(n_sc, 0) + 1
        if self.first_recv is None:
            self.first_recv = now
        self.last_recv = now

        if self.csv_w:
            row = [f"{now:.6f}", self.label]
            for f in HEADER_FIELDS[1:]:
                v = hdr[f]
                row.append(v.hex() if isinstance(v, bytes) else v)
            row.append(data[HEADER_SIZE:].hex())
            self.csv_w.writerow(row)

        return hdr

    def close(self):
        self.set_label(self.label, self.label_note)  # flush final block
        meta = {
            "magic": f"0x{MAGIC:08X}",
            "header_fmt": HEADER_FMT,
            "header_size": HEADER_SIZE,
            "header_fields": HEADER_FIELDS,
            "record_layout": "u32 rec_len | f64 recv_unix | u8 label_len | label | header+csi",
            "started_unix": self.started_unix,
            "ended_unix": time.time(),
            "frames": self.frames,
            "bytes": self.bytes_written,
            "lost_inferred": self.lost,
            "bad_magic": self.bad_magic,
            "short_packets": self.short,
            "frames_per_label": self.per_label,
            "per_node": {
                str(k): {
                    "frames": v["frames"], "lost": v["lost"],
                    "rate_hz": (v["frames"] / (v["last"] - v["first"])
                                if v["last"] > v["first"] else 0.0),
                } for k, v in self.per_node.items()
            },
            # Labels prefixed "transition_" are walking-into-position periods and
            # MUST be excluded from analysis: they contain motion that belongs to
            # neither the preceding nor following condition.
            "exclude_label_prefix": "transition_",
            "node_positions": self.node_positions,
            "subcarrier_counts": {str(k): v for k, v in self.sc_counts.items()},
            "mean_rate_hz": (
                self.frames / (self.last_recv - self.first_recv)
                if self.first_recv and self.last_recv and self.last_recv > self.first_recv
                else 0.0
            ),
        }
        with open(self.meta_path, "w") as f:
            json.dump(meta, f, indent=2)

        self.bin_f.close()
        self.labels_f.close()
        if self.csv_f:
            self.csv_f.close()
        return meta


class CSIProtocol(asyncio.DatagramProtocol):
    def __init__(self, rec: Recorder):
        self.rec = rec

    def datagram_received(self, data: bytes, addr):
        self.rec.add(data)


async def status_loop(rec: Recorder, interval: float = 2.0):
    """Print a live one-liner. Frame rate is the thing to watch: if it differs
    between conditions, that alone could explain an apparent 'signal'."""
    prev = 0
    while True:
        await asyncio.sleep(interval)
        n = rec.frames
        rate = (n - prev) / interval
        prev = n
        if rec.quiet:
            continue
        elapsed = time.time() - rec.label_started
        # Per-node counts, so a silent second node is obvious immediately rather
        # than discovered after the run.
        nodes = "/".join(f"n{k}:{v['frames']}" for k, v in sorted(rec.per_node.items()))
        sys.stdout.write(
            f"\r[{rec.label}] {elapsed:5.1f}s | {n:>7} frames | {rate:5.1f}/s "
            f"| lost {rec.lost} | {nodes or 'no nodes'}    "
        )
        sys.stdout.flush()


def beep(n: int = 1):
    """Terminal bell. The protocol has to be driven from outside the room for the
    'empty' blocks, so transitions must be audible rather than on-screen."""
    for _ in range(n):
        sys.stdout.write("\a")
        sys.stdout.flush()
        time.sleep(0.15)


async def run_protocol(rec: Recorder, protocol, lead_in: float):
    """Walk the interleaved protocol hands-free.

    Every block gets a spoken-aloud-style lead-in and audible beeps instead of
    requiring a keypress: several blocks need you OUT of the room, where you
    cannot press a key. One beep = get into position, two beeps = block started,
    three beeps = protocol done.
    """
    total = sum(d for _, d, _ in protocol) + lead_in * len(protocol)
    print(f"\nProtocol: {len(protocol)} blocks, ~{total/60:.1f} min including "
          f"{lead_in:.0f}s lead-in per block.")
    print("Fully hands-free: listen for the beeps. 1 beep = move into position,")
    print("2 beeps = recording that block, 3 beeps = all done.")
    print("Turn your terminal volume up. Ctrl-C stops early and still saves.\n")
    await asyncio.sleep(2)

    for i, (label, dur, note) in enumerate(protocol, 1):
        print(f"\n{'='*66}")
        print(f"  BLOCK {i}/{len(protocol)}: {label}   ({dur}s)")
        print(f"  {note}")
        print(f"{'='*66}")

        # Lead-in: time to walk to position. Recorded under a "transition_*"
        # label so the walking-to-position motion never contaminates the block.
        rec.set_label(f"transition_to_{label}", "moving into position; exclude from analysis")
        beep(1)
        rec.quiet = True
        for remaining in range(int(lead_in), 0, -1):
            sys.stdout.write(f"\r  get into position... {remaining:3d}s   ")
            sys.stdout.flush()
            await asyncio.sleep(1)
        rec.quiet = False

        before = rec.frames
        rec.set_label(label, note)
        beep(2)

        # Countdown so you know when to move without watching a separate clock.
        # The status line is suspended during the block to keep this readable.
        rec.quiet = True
        for remaining in range(dur, 0, -1):
            sys.stdout.write(f"\r  {label}: {remaining:3d}s remaining   "
                             f"({rec.frames - before} frames)      ")
            sys.stdout.flush()
            await asyncio.sleep(1)
        rec.quiet = False

        got = rec.frames - before
        rate = got / dur if dur else 0
        warn = "  <-- LOW, check the node!" if rate < 5 else ""
        print(f"\r  {label}: done. {got} frames at {rate:.1f}/s{warn}          ")

    beep(3)
    print(f"\n\n{'='*66}")
    print("  PROTOCOL COMPLETE -- saving automatically.")
    print(f"{'='*66}")


async def manual_labels(rec: Recorder):
    """Read label names from stdin while recording."""
    loop = asyncio.get_running_loop()
    print("\nType a label + ENTER to switch label. Ctrl-C to stop.\n")
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            break
        lbl = line.strip()
        if lbl:
            rec.set_label(lbl)
            print(f"  -> label now '{lbl}'")


def collect_node_positions(expected, existing=None):
    """Prompt for where each node physically is.

    Without positions, N nodes are just N uncalibrated sensors and no spatial
    inference is possible -- the geometry IS the information. Recorded into the
    sidecar so a capture stays interpretable months later.

    Coordinates are free-form (metres from a room corner is the easy convention);
    a description alone is still better than nothing.
    """
    if existing:
        print(f"\nreusing node positions from {existing}")
        return existing_positions(existing)

    print(f"\n{'='*66}")
    print("  NODE POSITIONS")
    print("  Enter where each node is. 'x,y' in metres from one room corner is")
    print("  ideal; a description alone is still useful. Blank to skip a node.")
    print(f"{'='*66}")
    pos = {}
    for nid in expected:
        raw = input(f"  node {nid} position (e.g. '0.5,2.1 NE corner on shelf'): ").strip()
        if raw:
            xy, desc = None, raw
            parts = raw.split(None, 1)
            if "," in parts[0]:
                try:
                    a, b = parts[0].split(",")[:2]
                    xy = [float(a), float(b)]
                    desc = parts[1] if len(parts) > 1 else ""
                except ValueError:
                    xy = None
            pos[str(nid)] = {"xy_m": xy, "note": desc}
    extra = input("  room notes (dimensions, wall A/B, AP location, spots 1-4): ").strip()
    if extra:
        pos["_room"] = {"note": extra}
    return pos


def existing_positions(path):
    try:
        with open(path) as f:
            return json.load(f).get("node_positions", {})
    except (OSError, ValueError):
        return {}


async def wait_for_nodes(rec, expected_ids, timeout=90):
    """Block until every expected node has reported, or timeout.

    With five boards, one silently failing is the likeliest way to waste a
    session; catching it before the protocol starts costs seconds.
    """
    print(f"\nwaiting for nodes {sorted(expected_ids)} to report...")
    waited = 0.0
    while waited < timeout:
        seen = set(rec.per_node)
        missing = set(expected_ids) - seen
        if not missing:
            print(f"  all {len(expected_ids)} nodes reporting: " +
                  ", ".join(f"n{k}={rec.per_node[k]['frames']}" for k in sorted(seen)))
            return True
        await asyncio.sleep(1.0)
        waited += 1.0
        if waited % 10 == 0:
            print(f"  {waited:.0f}s: have {sorted(seen) or 'none'}, "
                  f"still missing {sorted(missing)}")
    seen = set(rec.per_node)
    missing = set(expected_ids) - seen
    print(f"\n  !! TIMEOUT: nodes {sorted(missing)} never reported.")
    ans = input("  Continue with only the nodes present? [y/N]: ").strip().lower()
    return ans == "y"


def lan_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "localhost"
    finally:
        s.close()


async def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", required=True, help="output path prefix, e.g. data/session1")
    p.add_argument("--udp-host", default="0.0.0.0")
    p.add_argument("--udp-port", type=int, default=5005)
    p.add_argument("--protocol", action="store_true",
                   help="run the guided interleaved capture protocol")
    p.add_argument("--csv", action="store_true",
                   help="also write a (much larger) CSV alongside the binary")
    p.add_argument("--duration", type=float, default=0,
                   help="stop after N seconds (0 = until Ctrl-C)")
    p.add_argument("--lead-in", type=float, default=15,
                   help="seconds between blocks to walk into position (default 15)")
    p.add_argument("--label", default="check",
                   help="label to use with --duration (default 'check')")
    p.add_argument("--protocol-name", default="v2", choices=sorted(PROTOCOLS),
                   help="which protocol to run (default v2)")
    p.add_argument("--nodes", default="",
                   help="comma-separated node IDs expected, e.g. 1,2,3,4,5. "
                        "The run waits for all of them before starting.")
    p.add_argument("--positions-from",
                   help="reuse node_positions from a previous .json sidecar")
    p.add_argument("--no-positions", action="store_true",
                   help="skip the node-position prompts")
    args = p.parse_args()

    rec = Recorder(args.out, args.csv)
    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        lambda: CSIProtocol(rec), local_addr=(args.udp_host, args.udp_port)
    )

    print(f"Recording to {rec.bin_path}")
    print(f"Listening on {args.udp_host}:{args.udp_port} (this machine: {lan_ip()})")
    print("Point the ESP32's CSI_TARGET_IP at that address.")
    if args.csv:
        print(f"Also writing CSV: {rec.csv_path}")

    # Wait for real frames before prompting for the first block. This makes the
    # launch order irrelevant: start this before or after flashing, and the
    # protocol won't begin until data is genuinely arriving. It also catches the
    # common failure of still running the OLD firmware -- packets with a stale
    # magic are counted as bad_magic and never look like valid frames.
    print("\nWaiting for CSI frames from the ESP32...")
    waited = 0.0
    while rec.frames == 0:
        await asyncio.sleep(0.5)
        waited += 0.5
        if rec.bad_magic > 0 and waited % 3 == 0:
            print(f"  !! receiving packets but magic is wrong ({rec.bad_magic} so far) -- "
                  f"the ESP32 is probably running OLD firmware. Reflash it.")
        elif waited % 10 == 0:
            print(f"  ...still nothing after {waited:.0f}s. Check the ESP32 is powered, "
                  f"on Wi-Fi, and CSI_TARGET_IP points at {lan_ip()}.")
    print(f"Frames arriving. Good.\n")

    # Pre-flight: confirm every expected node is alive before committing to a
    # long protocol, and record where they are.
    expected = []
    if args.nodes:
        expected = [int(x) for x in args.nodes.split(",") if x.strip()]
        ok = await wait_for_nodes(rec, expected)
        if not ok:
            transport.close()
            rec.close()
            sys.exit("aborted: not all expected nodes present")

    if args.protocol and not args.no_positions:
        ids = expected or sorted(rec.per_node)
        loop2 = asyncio.get_running_loop()
        rec.node_positions = await loop2.run_in_executor(
            None, collect_node_positions, ids, args.positions_from)

    tasks = [asyncio.create_task(status_loop(rec))]
    main_task = None
    if args.protocol:
        proto = PROTOCOLS[args.protocol_name]
        print(f"\nusing protocol '{args.protocol_name}' "
              f"({len(proto)} blocks)")
        main_task = asyncio.create_task(run_protocol(rec, proto, args.lead_in))
        tasks.append(main_task)
    elif args.duration <= 0:
        # Only read stdin when we have no other stop condition. A blocking
        # readline() cannot be cancelled, and asyncio waits for executor threads
        # at shutdown, so starting it alongside --duration hangs the exit.
        main_task = asyncio.create_task(manual_labels(rec))
        tasks.append(main_task)

    if args.duration > 0:
        rec.set_label(args.label)

    try:
        if args.duration > 0:
            await asyncio.sleep(args.duration)
        elif main_task is not None:
            await main_task
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        for t in tasks:
            t.cancel()
        transport.close()
        meta = rec.close()
        print("\n\n=== capture summary ===")
        print(f"  frames        : {meta['frames']}")
        print(f"  mean rate     : {meta['mean_rate_hz']:.1f} Hz")
        print(f"  lost (seq gap): {meta['lost_inferred']}")
        print(f"  size          : {meta['bytes']/1048576:.1f} MB")
        print(f"  widths seen   : {meta['subcarrier_counts']}")
        if meta["per_node"]:
            print(f"  per node      :")
            for k, v in sorted(meta["per_node"].items()):
                print(f"      node {k}: {v['frames']} frames, "
                      f"{v['rate_hz']:.1f} Hz, {v['lost']} lost")
            if len(meta["per_node"]) == 1:
                print("      ^ only ONE node reported. If you expected two, the "
                      "second is not streaming.")
        print(f"  per label     :")
        for k, v in meta["frames_per_label"].items():
            print(f"      {k:<20} {v}")
        print(f"\n  data   : {rec.bin_path}")
        print(f"  meta   : {rec.meta_path}")
        print(f"  labels : {rec.labels_path}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
