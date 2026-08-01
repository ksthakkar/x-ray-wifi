"""
CSI listener + live web viewer for the test-node ESP32 firmware.

Receives ADR-018-framed UDP packets (magic 0xC5110001) from the ESP32,
prints a decoded summary per packet to the console, and serves a live
amplitude/RSSI/motion graph at http://localhost:8080.

The motion score is computed on the ESP32 (see the presence smoke test in
test-node/src/main.c) and carried in the ADR-018 header, so the firmware's
serial bar graph and this dashboard always show the same number.

Usage:
    pip install -r requirements.txt
    python csi_viewer.py [--udp-port 5005] [--http-port 8080]

Point the ESP32's CSI_TARGET_IP (in test-node/src/credentials.h) at the
machine running this script — normally the Arduino UNO Q on the same LAN,
so the dashboard is reachable at http://<uno-q-ip>:8080 from any device on
the network. A laptop works the same way for local dev.
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
latest_frame: dict = {}
packets_received = 0
packets_per_sec = 0.0


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

        # Keep this handler cheap: just record the newest frame and count it.
        # Anything expensive (JSON, sockets, stdout) happens on the timers below,
        # off the hot path.
        global latest_frame, packets_received
        latest_frame = frame
        frame["addr"] = addr[0]
        packets_received += 1


async def broadcast_loop():
    """Push the newest frame to browsers at a fixed rate, not per packet."""
    interval = 1.0 / BROADCAST_HZ
    last_sent_seq = None

    while True:
        await asyncio.sleep(interval)
        frame = latest_frame
        if not frame or not clients:
            continue
        # Nothing new since the last tick (ESP32 offline or slower than us).
        if frame["sequence"] == last_sent_seq:
            continue
        last_sent_seq = frame["sequence"]

        amps = frame["amplitudes"]
        motion = frame["motion"]
        payload = json.dumps(
            {
                "sequence": frame["sequence"],
                "node_id": frame["node_id"],
                "rssi": frame["rssi"],
                "noise_floor": frame["noise_floor"],
                "num_subcarriers": frame["num_subcarriers"],
                "amplitudes": amps,
                "amp_mean": (sum(amps) / len(amps)) if amps else 0.0,
                "motion": motion,
                "excess": frame["excess"],
                "csi_floor": frame["csi_floor"],
                "distance_m": frame["distance_m"],
                # Presence is judged on excess, not the raw score.
                "presence": (
                    frame["excess"] is not None
                    and frame["excess"] >= MOTION_PRESENCE_THRESHOLD
                ),
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
    """Periodic one-line summary, including the true packet rate."""
    global packets_per_sec, packets_received
    interval = 1.0 / CONSOLE_LOG_HZ if CONSOLE_LOG_HZ else 1.0
    prev_count = 0
    prev_seq = None
    lost_total = 0

    while True:
        await asyncio.sleep(interval)

        seen = packets_received - prev_count
        prev_count = packets_received
        packets_per_sec = seen / interval

        frame = latest_frame
        if not frame:
            continue

        # Gaps in the sequence counter mean packets were lost in flight (Wi-Fi
        # or the ESP32's own queue), which is the usual cause of a choppy graph.
        seq = frame["sequence"]
        if prev_seq is not None and seq > prev_seq:
            gap = seq - prev_seq - seen
            if gap > 0:
                lost_total += gap
        prev_seq = seq

        if not CONSOLE_LOG_HZ:
            continue

        amps = frame["amplitudes"]
        amp_mean = (sum(amps) / len(amps)) if amps else 0.0
        motion = frame["motion"]
        motion_str = "warmup" if motion is None else f"{motion:6.2f}"

        print(
            f"[{frame.get('addr', '?')}] seq={seq:>8} {packets_per_sec:5.1f} pkt/s "
            f"sc={frame['num_subcarriers']:>4} rssi={frame['rssi']:>4}dBm "
            f"amp_mean={amp_mean:6.1f} motion={motion_str} "
            f"lost={lost_total} clients={len(clients)}"
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
  h1 { font-size:1.1rem; font-weight:600; color:#9fd3ff; margin:0 0 1rem; }
  .stats { display:flex; gap:1.5rem; margin-bottom:1rem; flex-wrap:wrap; }
  .stat { background:#151923; border:1px solid #232838; border-radius:8px; padding:.6rem 1rem; min-width:110px; }
  .stat .label { font-size:.7rem; color:#8892a6; text-transform:uppercase; letter-spacing:.05em; }
  .stat .value { font-size:1.3rem; color:#e6e6e6; margin-top:.2rem; }
  canvas#radar { height:260px; }
  canvas { background:#11141c; border:1px solid #232838; border-radius:8px; width:100%; height:220px; display:block; margin-bottom:1.2rem; }
  .status { font-size:.8rem; color:#8892a6; }
  .status.live { color:#5ee6a0; }
  /* Motion / presence panel */
  .stat.presence { min-width:170px; }
  .stat.presence.on { border-color:#e6a35e; background:#20180f; }
  #presence { color:#8892a6; }
  .stat.presence.on #presence { color:#ffb765; }
  .meter { height:10px; background:#232838; border-radius:5px; overflow:hidden; margin-top:.45rem; }
  .meter-fill { height:100%; width:0%; background:#5ee6a0; border-radius:5px; transition:width .08s linear; }
  .meter-fill.on { background:#ffb765; }
  .section-label { margin-bottom:.4rem; color:#8892a6; font-size:.75rem; text-transform:uppercase; letter-spacing:.05em; }
</style>
</head>
<body>
<h1>x-ray-wifi &mdash; CSI Viewer</h1>
<div class="stats">
  <div class="stat"><div class="label">Status</div><div class="value status" id="status">waiting&hellip;</div></div>
  <div class="stat"><div class="label">Sequence</div><div class="value" id="seq">-</div></div>
  <div class="stat"><div class="label">Node</div><div class="value" id="node">-</div></div>
  <div class="stat"><div class="label">Subcarriers</div><div class="value" id="sc">-</div></div>
  <div class="stat"><div class="label">RSSI</div><div class="value" id="rssi">-</div></div>
  <div class="stat"><div class="label">Noise floor</div><div class="value" id="noise">-</div></div>
  <div class="stat"><div class="label">Packet rate</div><div class="value" id="pps">-</div></div>
  <div class="stat"><div class="label">Est. distance</div><div class="value" id="dist">-</div></div>
  <div class="stat presence" id="presenceCard">
    <div class="label">Motion / presence</div>
    <div class="value"><span id="motion">-</span> <span id="presence" style="font-size:.8rem;">&nbsp;</span></div>
    <div class="meter"><div class="meter-fill" id="motionFill"></div></div>
  </div>
</div>

<div class="section-label">Amplitude per subcarrier</div>
<canvas id="amp" height="220"></canvas>

<div class="section-label">Proximity &mdash; radius only; a single antenna carries no direction information</div>
<canvas id="radar" height="260"></canvas>

<div class="section-label">Excess over noise floor &mdash; flat when still, spikes when a hand covers the board</div>
<canvas id="motionHist" height="220"></canvas>

<div class="section-label">RSSI / mean amplitude history</div>
<canvas id="hist" height="220"></canvas>

<script>
const ampCanvas = document.getElementById('amp');
const histCanvas = document.getElementById('hist');
const motionCanvas = document.getElementById('motionHist');
const radarCanvas = document.getElementById('radar');
const statusEl = document.getElementById('status');

function fitCanvas(c) {
  const rect = c.getBoundingClientRect();
  c.width = rect.width * devicePixelRatio;
  c.height = rect.height * devicePixelRatio;
}
window.addEventListener('resize', () => { fitCanvas(ampCanvas); fitCanvas(histCanvas); fitCanvas(motionCanvas); fitCanvas(radarCanvas); });
fitCanvas(ampCanvas); fitCanvas(histCanvas); fitCanvas(motionCanvas); fitCanvas(radarCanvas);

const rssiHistory = [];
const ampHistory = [];
const motionHistory = [];
const HISTORY_LEN = 200;
// Meter is full at this score; also the floor for the history y-axis, so an
// idle trace stays visibly flat instead of auto-scaling noise to full height.
const MOTION_FULL_SCALE = 40;
let motionThreshold = null;

function drawAmplitudes(amps) {
  const ctx = ampCanvas.getContext('2d');
  const w = ampCanvas.width, h = ampCanvas.height;
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
const MAX_RADIUS_M = 4.0;
function drawRadar(distM) {
  const ctx = radarCanvas.getContext('2d');
  const w = radarCanvas.width, h = radarCanvas.height;
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2;
  const maxR = Math.min(w, h) / 2 - 24 * devicePixelRatio;

  // Range rings, labelled in metres.
  ctx.font = `${10 * devicePixelRatio}px ui-monospace, monospace`;
  for (let m = 1; m <= MAX_RADIUS_M; m++) {
    const r = (m / MAX_RADIUS_M) * maxR;
    ctx.strokeStyle = '#232838';
    ctx.lineWidth = 1 * devicePixelRatio;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#4a5164';
    ctx.fillText(m + 'm', cx + r - 12 * devicePixelRatio, cy - 3 * devicePixelRatio);
  }

  // The node itself.
  ctx.fillStyle = '#9fd3ff';
  ctx.beginPath(); ctx.arc(cx, cy, 4 * devicePixelRatio, 0, Math.PI * 2); ctx.fill();

  if (distM === null || distM === undefined) {
    ctx.fillStyle = '#4a5164';
    ctx.font = `${12 * devicePixelRatio}px ui-monospace, monospace`;
    ctx.fillText('no target', cx - 26 * devicePixelRatio, cy + 18 * devicePixelRatio);
    return;
  }

  // Detection annulus: the target is somewhere on this ring, at unknown bearing.
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

function drawMotionHistory() {
  const ctx = motionCanvas.getContext('2d');
  const w = motionCanvas.width, h = motionCanvas.height;
  ctx.clearRect(0, 0, w, h);
  if (motionHistory.length < 2) return;

  const max = Math.max(...motionHistory, MOTION_FULL_SCALE);
  const yOf = (v) => h - (v / max) * (h - 10) - 5;

  // Threshold line: above it, the server calls it presence.
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

  // Filled area under the trace, so spikes read at a glance.
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

function drawHistory() {
  const ctx = histCanvas.getContext('2d');
  const w = histCanvas.width, h = histCanvas.height;
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
  ws.onmessage = (ev) => { pendingFrame = JSON.parse(ev.data); };
}

let pendingFrame = null;

function render() {
  requestAnimationFrame(render);
  const f = pendingFrame;
  if (!f) return;
  pendingFrame = null;
  {
    document.getElementById('seq').textContent = f.sequence;
    document.getElementById('node').textContent = f.node_id;
    document.getElementById('sc').textContent = f.num_subcarriers;
    document.getElementById('rssi').textContent = f.rssi + ' dBm';
    document.getElementById('noise').textContent = f.noise_floor + ' dBm';

    if (f.motion_threshold !== undefined) motionThreshold = f.motion_threshold;

    const motionEl = document.getElementById('motion');
    const presenceEl = document.getElementById('presence');
    const cardEl = document.getElementById('presenceCard');
    const fillEl = document.getElementById('motionFill');

    // Display EXCESS (motion above the learned noise floor), not the raw score:
    // the raw idle level varies by environment so it isn't comparable.
    const distEl = document.getElementById('dist');
    if (f.excess === null || f.excess === undefined) {
      motionEl.textContent = '--';
      presenceEl.textContent = 'warming up';
      distEl.textContent = '-';
      cardEl.classList.remove('on');
      fillEl.classList.remove('on');
      fillEl.style.width = '0%';
      drawRadar(null);
    } else {
      motionEl.textContent = f.excess.toFixed(1);
      presenceEl.textContent = f.presence ? 'PRESENCE' : 'clear';
      cardEl.classList.toggle('on', !!f.presence);
      fillEl.classList.toggle('on', !!f.presence);
      fillEl.style.width = Math.min(100, (f.excess / MOTION_FULL_SCALE) * 100) + '%';

      distEl.textContent = (f.distance_m === null || f.distance_m === undefined)
        ? 'no target' : '~' + f.distance_m.toFixed(2) + ' m';

      motionHistory.push(f.excess);
      if (motionHistory.length > HISTORY_LEN) motionHistory.shift();
      drawMotionHistory();
      drawRadar(f.distance_m);
    }

    drawAmplitudes(f.amplitudes);

    rssiHistory.push(f.rssi);
    ampHistory.push(f.amp_mean);
    if (rssiHistory.length > HISTORY_LEN) rssiHistory.shift();
    if (ampHistory.length > HISTORY_LEN) ampHistory.shift();
    drawHistory();

    if (f.pps !== undefined) document.getElementById('pps').textContent = f.pps.toFixed(0) + '/s';
  }
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
