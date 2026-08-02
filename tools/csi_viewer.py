"""
CSI listener + live web viewer for the test-node ESP32 firmware.

Receives ADR-018-framed UDP packets (magic 0xC5110001) from one or more
ESP32 nodes (distinguished by the header's node_id), prints a decoded
summary per packet to the console, and serves a live per-node dashboard
(amplitude/RSSI/motion/distance) at http://localhost:8080.

The motion score is computed on the ESP32 (see the presence smoke test in
test-node/src/main.c) and carried in the ADR-018 header, so the firmware's
serial bar graph and this dashboard always show the same number.

Usage:
    pip install -r requirements.txt
    python csi_viewer.py [--udp-port 5005] [--http-port 8080]

Point each ESP32's CSI_TARGET_IP (in test-node/src/credentials.h) at the
machine running this script — normally the Arduino UNO Q on the same LAN,
so the dashboard is reachable at http://<uno-q-ip>:8080 from any device on
the network. A laptop works the same way for local dev. Give each physical
node a distinct CSI_NODE_ID in its own credentials.h so this viewer can tell
their streams apart.
"""

import argparse
import asyncio
import json
import math
import socket
import struct
import time

from aiohttp import web

MAGIC = 0xC5110001
# Original 20-byte ADR-018 header, extended with the presence fields the
# firmware now computes on-device (excess over noise floor, and pseudo-distance).
HEADER_FMT = "<IBBHIIbbHHHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 26, HEADER_SIZE

# Sentinel: firmware sees nothing above its noise floor.
DISTANCE_NO_TARGET = 0xFFFF

# Header sentinel: firmware is still building its amplitude baseline.
MOTION_WARMING_UP = 0xFFFF

# Presence threshold, applied to EXCESS over the firmware's learned noise floor
# -- not to the raw score. The raw idle level depends on ambient Wi-Fi traffic
# (observed anywhere from ~1 to ~15), so thresholding it directly gives constant
# false positives. Excess is ~0 when idle regardless of environment.
MOTION_PRESENCE_THRESHOLD = 4.0

# The ESP32 sends at ~50 Hz. Encoding JSON and pushing a WebSocket frame per
# packet -- and redrawing three canvases per packet in the browser -- is far more
# than a display needs, and it makes the page lag. Instead the UDP handler only
# stores the newest frame, and a timer broadcasts at this rate. Dropping stale
# frames is the right trade for a live view: we always show the latest state.
BROADCAST_HZ = 15

# Console printing is the other bottleneck: stdout is line-buffered and blocking,
# so a print() per packet stalls the event loop. Print a summary this often
# instead (0 disables per-packet logging entirely).
CONSOLE_LOG_HZ = 2

clients: set[web.WebSocketResponse] = set()
# Keyed by node_id, so multiple physical nodes don't overwrite each other's
# state -- each keeps its own newest frame, independent of the others.
latest_frames: dict[int, dict] = {}
packets_received = 0
packets_per_sec = 0.0

# Heavier smoothing than the firmware's own (fast, jitter-only) EMA, applied
# here purely for display/detection stability at the cost of a bit of lag.
# Runs at the full ~50 Hz packet rate, not BROADCAST_HZ, so these alphas are
# tuned for that rate.
AMP_EMA_ALPHA = 0.15
AMP_BAR_EMA_ALPHA = 0.2
EXCESS_EMA_ALPHA = 0.12
DISTANCE_EMA_ALPHA = 0.15

# Hysteresis on presence: enter at the normal threshold, but require dropping
# well below it to clear. Without this, excess hovering near the threshold
# flickers PRESENCE on/off every tick even after EMA smoothing.
PRESENCE_ON = MOTION_PRESENCE_THRESHOLD
PRESENCE_OFF = MOTION_PRESENCE_THRESHOLD * 0.5

# Per-node EMA/hysteresis state, keyed by node_id -- independent of the
# per-node baselines each ESP32 already tracks on-device.
smoothing_state: dict[int, dict] = {}


def parse_packet(data: bytes):
    if len(data) < HEADER_SIZE:
        return None

    (
        magic, node_id, num_antennas, num_subcarriers, freq_mhz, sequence,
        rssi, noise_floor, motion_q8, excess_q8, floor_q8, distance_cm,
    ) = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    if magic != MAGIC:
        return None

    # Fixed-point (value * 256); MOTION_WARMING_UP means the baseline isn't ready.
    motion = None if motion_q8 == MOTION_WARMING_UP else motion_q8 / 256.0
    excess = None if excess_q8 == MOTION_WARMING_UP else excess_q8 / 256.0
    csi_floor = None if floor_q8 == MOTION_WARMING_UP else floor_q8 / 256.0
    distance_m = None if distance_cm == DISTANCE_NO_TARGET else distance_cm / 100.0

    csi_bytes = data[HEADER_SIZE:]
    n_pairs = len(csi_bytes) // 2
    amplitudes = []
    phases = []
    for i in range(n_pairs):
        im, re = struct.unpack_from("<bb", csi_bytes, i * 2)
        amplitudes.append(math.hypot(re, im))
        phases.append(math.atan2(im, re))

    return {
        "node_id": node_id,
        "num_antennas": num_antennas,
        "num_subcarriers": num_subcarriers,
        "freq_mhz": freq_mhz,
        "sequence": sequence,
        "rssi": rssi,
        "noise_floor": noise_floor,
        "motion": motion,
        "excess": excess,
        "csi_floor": csi_floor,
        "distance_m": distance_m,
        "amplitudes": amplitudes,
        "phases": phases,
        "ts": time.time(),
    }


class CSIProtocol(asyncio.DatagramProtocol):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop

    def datagram_received(self, data: bytes, addr):
        frame = parse_packet(data)
        if frame is None:
            return

        # Keep this handler cheap: just record the newest frame per node and
        # count it. Anything expensive (JSON, sockets, stdout) happens on the
        # timers below, off the hot path.
        global packets_received
        frame["addr"] = addr[0]
        apply_smoothing(frame)
        latest_frames[frame["node_id"]] = frame
        packets_received += 1


def apply_smoothing(frame: dict):
    """Heavily smooth amplitude/excess/distance and add hysteresis to presence.

    Mutates frame in place, adding smoothed values under new keys so the raw
    on-device numbers stay available (console log, diagnostics) untouched.
    Runs per packet, at the ESP32's native rate, so it settles fast despite
    the low alphas -- BROADCAST_HZ only throttles what reaches the browser,
    not how often this filter updates.
    """
    node_id = frame["node_id"]
    state = smoothing_state.get(node_id)
    amps = frame["amplitudes"]
    amp_mean = (sum(amps) / len(amps)) if amps else 0.0

    # Frame width (subcarrier count) can change frame-to-frame (interleaved
    # non-HT/HT frames -- see main.c's presence_slot logic), so a per-index
    # amplitude EMA only makes sense while width is stable; reset on change.
    if state is not None and len(state.get("amplitudes", [])) != len(amps):
        state = None

    if state is None or frame["excess"] is None:
        # First packet for this node/width, or firmware still warming up its
        # own baseline: nothing to smooth against yet, just pass values through.
        state = {
            "amp_mean": amp_mean,
            "amplitudes": list(amps),
            "excess": frame["excess"] or 0.0,
            "distance_m": frame["distance_m"],
            "presence": False,
        }
        smoothing_state[node_id] = state
        frame["amp_mean_smooth"] = amp_mean
        frame["amplitudes_smooth"] = amps
        frame["excess_smooth"] = frame["excess"]
        frame["distance_m_smooth"] = frame["distance_m"]
        frame["presence_smooth"] = False
        return

    state["amp_mean"] += AMP_EMA_ALPHA * (amp_mean - state["amp_mean"])
    smoothed_amps = state["amplitudes"]
    for i, a in enumerate(amps):
        smoothed_amps[i] += AMP_BAR_EMA_ALPHA * (a - smoothed_amps[i])
    state["excess"] += EXCESS_EMA_ALPHA * (frame["excess"] - state["excess"])

    if frame["distance_m"] is None:
        # No target this packet -- hold the last smoothed distance rather
        # than snapping to "no target" instantly, so one dropped detection
        # doesn't blank the radar for a frame in an otherwise steady reading.
        pass
    elif state["distance_m"] is None:
        state["distance_m"] = frame["distance_m"]
    else:
        state["distance_m"] += DISTANCE_EMA_ALPHA * (frame["distance_m"] - state["distance_m"])

    # Hysteresis: separate on/off thresholds so smoothed excess hovering near
    # the boundary doesn't flip presence every tick.
    if state["presence"]:
        state["presence"] = state["excess"] >= PRESENCE_OFF
    else:
        state["presence"] = state["excess"] >= PRESENCE_ON

    # Once smoothed presence clears, drop the held distance too -- otherwise
    # a stale reading would linger on the radar after someone actually left.
    if not state["presence"]:
        state["distance_m"] = None

    frame["amp_mean_smooth"] = state["amp_mean"]
    frame["amplitudes_smooth"] = list(smoothed_amps)
    frame["excess_smooth"] = state["excess"]
    frame["distance_m_smooth"] = state["distance_m"]
    frame["presence_smooth"] = state["presence"]


def compute_fusion(frames: dict[int, dict]):
    """Combine all nodes' presence/distance into one confidence number.

    Deliberately not trilateration -- we don't know node positions, so this
    can't produce a room position. Instead it's agreement voting: presence is
    only declared fused-true when a majority of currently-calibrated nodes
    individually agree, which suppresses a single node's spurious excess
    spike (draft, reflection) from reading as a person. With only two nodes,
    "majority" is weak (1-of-2 already trips it) -- confidence is reported
    alongside so the UI can show that distinction instead of hiding it.
    """
    active = [f for f in frames.values() if f is not None]
    if not active:
        return None

    # Nodes still building their baseline don't get a vote either way --
    # counting them as "no presence" would bias fusion toward false negatives
    # every time a node restarts. Votes use the smoothed+hysteresis presence,
    # not raw excess, so fusion inherits the same flicker suppression.
    voting = [f for f in active if f["excess"] is not None]
    node_presence = [f["presence_smooth"] for f in voting]
    voting_count = len(voting)
    agree_count = sum(node_presence)

    fused_presence = voting_count > 0 and agree_count >= math.ceil(voting_count / 2)
    confidence = (agree_count / voting_count) if voting_count > 0 else 0.0

    distances = [
        f["distance_m_smooth"]
        for f, present in zip(voting, node_presence)
        if present and f["distance_m_smooth"] is not None
    ]
    fused_distance_m = (sum(distances) / len(distances)) if distances else None

    return {
        "presence": fused_presence,
        "confidence": confidence,
        "distance_m": fused_distance_m,
        "agree_count": agree_count,
        "voting_count": voting_count,
        "node_count": len(active),
    }


async def broadcast_loop():
    """Push the newest frame per node to browsers at a fixed rate, not per packet."""
    interval = 1.0 / BROADCAST_HZ
    last_sent_seq: dict[int, int] = {}

    while True:
        await asyncio.sleep(interval)
        if not latest_frames or not clients:
            continue

        nodes = []
        for node_id, frame in latest_frames.items():
            # Nothing new since the last tick for this node (offline or
            # slower than us) -- skip it, but still send the others.
            if frame["sequence"] == last_sent_seq.get(node_id):
                continue
            last_sent_seq[node_id] = frame["sequence"]

            amps = frame["amplitudes"]
            nodes.append(
                {
                    "node_id": node_id,
                    "sequence": frame["sequence"],
                    "rssi": frame["rssi"],
                    "noise_floor": frame["noise_floor"],
                    "num_subcarriers": frame["num_subcarriers"],
                    "amplitudes": frame.get("amplitudes_smooth", amps),
                    "amp_mean": frame["amp_mean_smooth"],
                    "motion": frame["motion"],
                    "excess": frame["excess_smooth"],
                    "csi_floor": frame["csi_floor"],
                    "distance_m": frame["distance_m_smooth"],
                    "presence": frame["presence_smooth"],
                }
            )
        if not nodes:
            continue

        payload = json.dumps(
            {
                "nodes": nodes,
                "fusion": compute_fusion(latest_frames),
                "motion_threshold": MOTION_PRESENCE_THRESHOLD,
                "pps": packets_per_sec,
            }
        )

        dead = []
        for ws in clients:
            try:
                await ws.send_str(payload)
            except (ConnectionResetError, ConnectionError):
                dead.append(ws)
        for ws in dead:
            clients.discard(ws)


async def console_loop():
    """Periodic one-line summary per node, including the true packet rate."""
    global packets_per_sec, packets_received
    interval = 1.0 / CONSOLE_LOG_HZ if CONSOLE_LOG_HZ else 1.0
    prev_count = 0
    prev_seq: dict[int, int] = {}
    lost_total: dict[int, int] = {}

    while True:
        await asyncio.sleep(interval)

        seen = packets_received - prev_count
        prev_count = packets_received
        packets_per_sec = seen / interval

        if not latest_frames:
            continue

        if not CONSOLE_LOG_HZ:
            continue

        for node_id, frame in sorted(latest_frames.items()):
            # Gaps in the sequence counter mean packets were lost in flight
            # (Wi-Fi or the ESP32's own queue), which is the usual cause of a
            # choppy graph. Tracked per node, since each node's sequence
            # counter is independent.
            seq = frame["sequence"]
            prev = prev_seq.get(node_id)
            if prev is not None and seq > prev:
                gap = seq - prev - 1
                if gap > 0:
                    lost_total[node_id] = lost_total.get(node_id, 0) + gap
            prev_seq[node_id] = seq

            amps = frame["amplitudes"]
            amp_mean = (sum(amps) / len(amps)) if amps else 0.0
            motion = frame["motion"]
            motion_str = "warmup" if motion is None else f"{motion:6.2f}"

            print(
                f"[node {node_id} {frame.get('addr', '?')}] seq={seq:>8} "
                f"{packets_per_sec:5.1f} pkt/s sc={frame['num_subcarriers']:>4} "
                f"rssi={frame['rssi']:>4}dBm amp_mean={amp_mean:6.1f} "
                f"motion={motion_str} lost={lost_total.get(node_id, 0)} "
                f"clients={len(clients)}"
            )


def lan_ip() -> str:
    """Best-effort LAN address of this machine, for the ESP32 and other laptops.

    Uses a UDP socket to a public address to pick the default-route interface;
    no packets are actually sent. Falls back to localhost if there's no route.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "localhost"
    finally:
        s.close()


async def index(_request: web.Request):
    return web.Response(text=INDEX_HTML, content_type="text/html")


async def ws_handler(request: web.Request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    clients.add(ws)
    try:
        async for _ in ws:
            pass
    finally:
        clients.discard(ws)
    return ws


INDEX_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>CSI Viewer</title>
<style>
  body { background:#0b0d12; color:#e6e6e6; font-family: ui-monospace, Consolas, monospace; margin:0; padding:1.5rem; }
  h1 { font-size:1.1rem; font-weight:600; color:#9fd3ff; margin:0 0 .25rem; }
  .global { display:flex; gap:1rem; margin-bottom:1rem; font-size:.8rem; color:#8892a6; }
  .status.live { color:#5ee6a0; }
  .node-panel { border:1px solid #232838; border-radius:10px; padding:1rem; margin-bottom:1.5rem; }
  .node-panel h2 { font-size:.95rem; margin:0 0 .75rem; color:#9fd3ff; }
  .stats { display:flex; gap:1.5rem; margin-bottom:1rem; flex-wrap:wrap; }
  .stat { background:#151923; border:1px solid #232838; border-radius:8px; padding:.6rem 1rem; min-width:110px; }
  .stat .label { font-size:.7rem; color:#8892a6; text-transform:uppercase; letter-spacing:.05em; }
  .stat .value { font-size:1.3rem; color:#e6e6e6; margin-top:.2rem; }
  canvas.radar { height:260px; }
  canvas { background:#11141c; border:1px solid #232838; border-radius:8px; width:100%; height:220px; display:block; margin-bottom:1.2rem; }
  /* Motion / presence panel */
  .stat.presence { min-width:170px; }
  .stat.presence.on { border-color:#e6a35e; background:#20180f; }
  .stat.presence .presence-label { color:#8892a6; }
  .stat.presence.on .presence-label { color:#ffb765; }
  .meter { height:10px; background:#232838; border-radius:5px; overflow:hidden; margin-top:.45rem; }
  .meter-fill { height:100%; width:0%; background:#5ee6a0; border-radius:5px; transition:width .08s linear; }
  .meter-fill.on { background:#ffb765; }
  .section-label { margin-bottom:.4rem; color:#8892a6; font-size:.75rem; text-transform:uppercase; letter-spacing:.05em; }
  /* Fused summary -- combined confidence across all nodes, no position data */
  .fusion-panel { border:1px solid #232838; border-radius:10px; padding:1rem; margin-bottom:1.5rem; display:none; }
  .fusion-panel.show { display:block; }
  .fusion-panel.on { border-color:#e6a35e; background:#20180f; }
  .fusion-panel h2 { font-size:.95rem; margin:0 0 .75rem; color:#9fd3ff; }
</style>
</head>
<body>
<h1>x-ray-wifi &mdash; CSI Viewer</h1>
<div class="global">
  <span>Status: <span class="status" id="status">waiting&hellip;</span></span>
  <span>Packet rate: <span id="pps">-</span></span>
  <span>Nodes seen: <span id="nodeCount">0</span></span>
</div>
<div id="panels"></div>

<div class="fusion-panel" id="fusionPanel">
  <h2>Fused presence (all nodes)</h2>
  <div class="stats">
    <div class="stat"><div class="label">Presence</div><div class="value" id="fusionPresence">-</div></div>
    <div class="stat"><div class="label">Agreement</div><div class="value" id="fusionAgreement">-</div></div>
    <div class="stat"><div class="label">Est. distance</div><div class="value" id="fusionDistance">-</div></div>
    <div class="stat presence">
      <div class="label">Confidence</div>
      <div class="value"><span id="fusionConfidence">-</span></div>
      <div class="meter"><div class="meter-fill" id="fusionMeterFill"></div></div>
    </div>
  </div>
</div>

<template id="panelTemplate">
<div class="node-panel">
  <h2>Node <span class="node-id-label"></span></h2>
  <div class="stats">
    <div class="stat"><div class="label">Sequence</div><div class="value seq">-</div></div>
    <div class="stat"><div class="label">Subcarriers</div><div class="value sc">-</div></div>
    <div class="stat"><div class="label">RSSI</div><div class="value rssi">-</div></div>
    <div class="stat"><div class="label">Est. distance</div><div class="value dist">-</div></div>
    <div class="stat presence">
      <div class="label">Motion / presence</div>
      <div class="value"><span class="motion">-</span> <span class="presence-label" style="font-size:.8rem;">&nbsp;</span></div>
      <div class="meter"><div class="meter-fill"></div></div>
    </div>
  </div>
  <div class="section-label">Amplitude per subcarrier</div>
  <canvas class="amp" height="220"></canvas>
  <div class="section-label">Proximity &mdash; radius only; a single antenna carries no direction information</div>
  <canvas class="radar" height="260"></canvas>
  <div class="section-label">Excess over noise floor &mdash; flat when still, spikes when a hand covers the board</div>
  <canvas class="motionHist" height="220"></canvas>
  <div class="section-label">RSSI / mean amplitude history</div>
  <canvas class="hist" height="220"></canvas>
</div>
</template>

<script>
const statusEl = document.getElementById('status');
const panelsEl = document.getElementById('panels');
const panelTemplate = document.getElementById('panelTemplate');
const HISTORY_LEN = 200;
// Meter is full at this score; also the floor for the history y-axis, so an
// idle trace stays visibly flat instead of auto-scaling noise to full height.
const MOTION_FULL_SCALE = 40;
const MAX_RADIUS_M = 4.0;
let motionThreshold = null;

// Per-node state: DOM refs + rolling history. Built lazily the first time a
// node_id is seen, so the page works with any number of nodes with no
// hardcoded assumption about how many there are.
const nodePanels = new Map();

function fitCanvas(c) {
  const rect = c.getBoundingClientRect();
  c.width = rect.width * devicePixelRatio;
  c.height = rect.height * devicePixelRatio;
}

function createPanel(nodeId) {
  const frag = panelTemplate.content.cloneNode(true);
  const root = frag.querySelector('.node-panel');
  root.querySelector('.node-id-label').textContent = nodeId;
  panelsEl.appendChild(root);

  const canvases = {
    amp: root.querySelector('canvas.amp'),
    radar: root.querySelector('canvas.radar'),
    motionHist: root.querySelector('canvas.motionHist'),
    hist: root.querySelector('canvas.hist'),
  };
  Object.values(canvases).forEach(fitCanvas);
  window.addEventListener('resize', () => Object.values(canvases).forEach(fitCanvas));

  return {
    root,
    canvases,
    els: {
      seq: root.querySelector('.seq'),
      sc: root.querySelector('.sc'),
      rssi: root.querySelector('.rssi'),
      dist: root.querySelector('.dist'),
      motion: root.querySelector('.motion'),
      presence: root.querySelector('.presence-label'),
      presenceCard: root.querySelector('.stat.presence'),
      motionFill: root.querySelector('.meter-fill'),
    },
    rssiHistory: [],
    ampHistory: [],
    motionHistory: [],
  };
}

function getPanel(nodeId) {
  let p = nodePanels.get(nodeId);
  if (!p) {
    p = createPanel(nodeId);
    nodePanels.set(nodeId, p);
    document.getElementById('nodeCount').textContent = nodePanels.size;
  }
  return p;
}

function drawAmplitudes(ctx, w, h, amps) {
  ctx.clearRect(0, 0, w, h);
  if (!amps.length) return;
  const max = Math.max(...amps, 1);
  const barW = w / amps.length;
  for (let i = 0; i < amps.length; i++) {
    const barH = (amps[i] / max) * (h - 10);
    const hue = 200 - (amps[i] / max) * 120;
    ctx.fillStyle = `hsl(${hue}, 80%, 60%)`;
    ctx.fillRect(i * barW, h - barH, Math.max(barW - 1, 1), barH);
  }
}

// Proximity ring. Deliberately a full circle, not a blip: one antenna gives
// range-like information only, so drawing a direction would be a fiction.
function drawRadar(ctx, w, h, distM) {
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2;
  const maxR = Math.min(w, h) / 2 - 24 * devicePixelRatio;

  ctx.font = `${10 * devicePixelRatio}px ui-monospace, monospace`;
  for (let m = 1; m <= MAX_RADIUS_M; m++) {
    const r = (m / MAX_RADIUS_M) * maxR;
    ctx.strokeStyle = '#232838';
    ctx.lineWidth = 1 * devicePixelRatio;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#4a5164';
    ctx.fillText(m + 'm', cx + r - 12 * devicePixelRatio, cy - 3 * devicePixelRatio);
  }

  ctx.fillStyle = '#9fd3ff';
  ctx.beginPath(); ctx.arc(cx, cy, 4 * devicePixelRatio, 0, Math.PI * 2); ctx.fill();

  if (distM === null || distM === undefined) {
    ctx.fillStyle = '#4a5164';
    ctx.font = `${12 * devicePixelRatio}px ui-monospace, monospace`;
    ctx.fillText('no target', cx - 26 * devicePixelRatio, cy + 18 * devicePixelRatio);
    return;
  }

  const r = Math.min(distM / MAX_RADIUS_M, 1) * maxR;
  const band = 14 * devicePixelRatio;
  const grad = ctx.createRadialGradient(cx, cy, Math.max(r - band, 0), cx, cy, r + band);
  grad.addColorStop(0, 'rgba(255,183,101,0)');
  grad.addColorStop(0.5, 'rgba(255,183,101,0.55)');
  grad.addColorStop(1, 'rgba(255,183,101,0)');
  ctx.fillStyle = grad;
  ctx.beginPath(); ctx.arc(cx, cy, r + band, 0, Math.PI * 2); ctx.fill();

  ctx.strokeStyle = '#ffb765';
  ctx.lineWidth = 2 * devicePixelRatio;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();

  ctx.fillStyle = '#ffb765';
  ctx.font = `${12 * devicePixelRatio}px ui-monospace, monospace`;
  ctx.fillText('~' + distM.toFixed(2) + ' m', cx - 24 * devicePixelRatio, cy + 18 * devicePixelRatio);
}

function drawMotionHistory(ctx, w, h, motionHistory) {
  ctx.clearRect(0, 0, w, h);
  if (motionHistory.length < 2) return;

  const max = Math.max(...motionHistory, MOTION_FULL_SCALE);
  const yOf = (v) => h - (v / max) * (h - 10) - 5;

  if (motionThreshold !== null) {
    ctx.strokeStyle = '#e6a35e';
    ctx.setLineDash([6 * devicePixelRatio, 6 * devicePixelRatio]);
    ctx.lineWidth = 1 * devicePixelRatio;
    ctx.beginPath();
    ctx.moveTo(0, yOf(motionThreshold));
    ctx.lineTo(w, yOf(motionThreshold));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const stepX = w / (HISTORY_LEN - 1);
  const startI = HISTORY_LEN - motionHistory.length;

  ctx.beginPath();
  ctx.moveTo(startI * stepX, h);
  motionHistory.forEach((v, i) => ctx.lineTo((startI + i) * stepX, yOf(v)));
  ctx.lineTo((startI + motionHistory.length - 1) * stepX, h);
  ctx.closePath();
  ctx.fillStyle = 'rgba(94, 230, 160, 0.15)';
  ctx.fill();

  ctx.strokeStyle = '#5ee6a0';
  ctx.lineWidth = 2 * devicePixelRatio;
  ctx.beginPath();
  motionHistory.forEach((v, i) => {
    const x = (startI + i) * stepX, y = yOf(v);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.fillStyle = '#8892a6';
  ctx.font = `${11 * devicePixelRatio}px ui-monospace, monospace`;
  ctx.fillText(max.toFixed(0), 4 * devicePixelRatio, 14 * devicePixelRatio);
}

function drawHistory(ctx, w, h, rssiHistory, ampHistory) {
  ctx.clearRect(0, 0, w, h);

  function drawSeries(data, color, min, max) {
    if (data.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2 * devicePixelRatio;
    ctx.beginPath();
    const stepX = w / (HISTORY_LEN - 1);
    const startI = HISTORY_LEN - data.length;
    data.forEach((v, i) => {
      const x = (startI + i) * stepX;
      const norm = (v - min) / (max - min || 1);
      const y = h - norm * (h - 10) - 5;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  drawSeries(rssiHistory, '#5ee6a0', -100, -20);
  drawSeries(ampHistory, '#9fd3ff', 0, Math.max(...ampHistory, 10));
}

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => { statusEl.textContent = 'live'; statusEl.classList.add('live'); };
  ws.onclose = () => {
    statusEl.textContent = 'disconnected, retrying...';
    statusEl.classList.remove('live');
    setTimeout(connect, 1000);
  };

  // Store the frame and let requestAnimationFrame do the drawing. Rendering
  // inside onmessage means the canvases redraw as fast as messages arrive,
  // which is what makes the page feel laggy.
  ws.onmessage = (ev) => { pendingPayload = JSON.parse(ev.data); };
}

let pendingPayload = null;

function renderNode(f) {
  const p = getPanel(f.node_id);
  const { els, canvases } = p;

  els.seq.textContent = f.sequence;
  els.sc.textContent = f.num_subcarriers;
  els.rssi.textContent = f.rssi + ' dBm';

  // Display EXCESS (motion above the learned noise floor), not the raw score:
  // the raw idle level varies by environment so it isn't comparable.
  if (f.excess === null || f.excess === undefined) {
    els.motion.textContent = '--';
    els.presence.textContent = 'warming up';
    els.dist.textContent = '-';
    els.presenceCard.classList.remove('on');
    els.motionFill.classList.remove('on');
    els.motionFill.style.width = '0%';
    drawRadar(canvases.radar.getContext('2d'), canvases.radar.width, canvases.radar.height, null);
  } else {
    els.motion.textContent = f.excess.toFixed(1);
    els.presence.textContent = f.presence ? 'PRESENCE' : 'clear';
    els.presenceCard.classList.toggle('on', !!f.presence);
    els.motionFill.classList.toggle('on', !!f.presence);
    els.motionFill.style.width = Math.min(100, (f.excess / MOTION_FULL_SCALE) * 100) + '%';

    els.dist.textContent = (f.distance_m === null || f.distance_m === undefined)
      ? 'no target' : '~' + f.distance_m.toFixed(2) + ' m';

    p.motionHistory.push(f.excess);
    if (p.motionHistory.length > HISTORY_LEN) p.motionHistory.shift();
    drawMotionHistory(canvases.motionHist.getContext('2d'), canvases.motionHist.width, canvases.motionHist.height, p.motionHistory);
    drawRadar(canvases.radar.getContext('2d'), canvases.radar.width, canvases.radar.height, f.distance_m);
  }

  drawAmplitudes(canvases.amp.getContext('2d'), canvases.amp.width, canvases.amp.height, f.amplitudes);

  p.rssiHistory.push(f.rssi);
  p.ampHistory.push(f.amp_mean);
  if (p.rssiHistory.length > HISTORY_LEN) p.rssiHistory.shift();
  if (p.ampHistory.length > HISTORY_LEN) p.ampHistory.shift();
  drawHistory(canvases.hist.getContext('2d'), canvases.hist.width, canvases.hist.height, p.rssiHistory, p.ampHistory);
}

function renderFusion(fusion) {
  const panel = document.getElementById('fusionPanel');
  if (!fusion) {
    panel.classList.remove('show');
    return;
  }
  panel.classList.add('show');
  panel.classList.toggle('on', !!fusion.presence);

  document.getElementById('fusionPresence').textContent = fusion.presence ? 'PRESENCE' : 'clear';
  document.getElementById('fusionAgreement').textContent = `${fusion.agree_count}/${fusion.voting_count} nodes`;
  document.getElementById('fusionDistance').textContent = (fusion.distance_m === null || fusion.distance_m === undefined)
    ? '-' : '~' + fusion.distance_m.toFixed(2) + ' m';
  document.getElementById('fusionConfidence').textContent = (fusion.confidence * 100).toFixed(0) + '%';

  const fill = document.getElementById('fusionMeterFill');
  fill.classList.toggle('on', !!fusion.presence);
  fill.style.width = Math.min(100, fusion.confidence * 100) + '%';
}

function render() {
  requestAnimationFrame(render);
  const payload = pendingPayload;
  if (!payload) return;
  pendingPayload = null;

  if (payload.motion_threshold !== undefined) motionThreshold = payload.motion_threshold;
  if (payload.pps !== undefined) document.getElementById('pps').textContent = payload.pps.toFixed(0) + '/s';

  payload.nodes.forEach(renderNode);
  renderFusion(payload.fusion);
}
connect();
render();
</script>
</body>
</html>
"""


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--udp-host", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    args = parser.parse_args()

    loop = asyncio.get_running_loop()

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.http_host, args.http_port)
    await site.start()

    transport, _protocol = await loop.create_datagram_endpoint(
        lambda: CSIProtocol(loop),
        local_addr=(args.udp_host, args.udp_port),
    )

    tasks = [
        asyncio.create_task(broadcast_loop()),
        asyncio.create_task(console_loop()),
    ]

    print(f"UDP listening on {args.udp_host}:{args.udp_port}")
    print(f"Web viewer at    http://{lan_ip()}:{args.http_port}")
    print(f"Broadcasting to browsers at {BROADCAST_HZ} Hz")
    print("Point the ESP32's CSI_TARGET_IP at this machine's LAN address above.")

    try:
        await asyncio.Event().wait()
    finally:
        for t in tasks:
            t.cancel()
        transport.close()
        await runner.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
