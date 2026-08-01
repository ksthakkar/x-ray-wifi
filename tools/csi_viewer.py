"""
CSI listener + live web viewer for the test-node ESP32 firmware.

Receives ADR-018-framed UDP packets (magic 0xC5110001) from the ESP32,
prints a decoded summary per packet to the console, and serves a live
amplitude/RSSI graph at http://localhost:8080.

Usage:
    pip install -r requirements.txt
    python csi_viewer.py [--udp-port 5005] [--http-port 8080]

Point the ESP32's CSI_TARGET_IP (in test-node/src/credentials.h) at the
machine running this script.
"""

import argparse
import asyncio
import json
import math
import struct
import time

from aiohttp import web

MAGIC = 0xC5110001
HEADER_FMT = "<IBBHIIbbH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 20, HEADER_SIZE

clients: set[web.WebSocketResponse] = set()
latest_frame: dict = {}


def parse_packet(data: bytes):
    if len(data) < HEADER_SIZE:
        return None

    magic, node_id, num_antennas, num_subcarriers, freq_mhz, sequence, rssi, noise_floor, _reserved = (
        struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    )
    if magic != MAGIC:
        return None

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

        global latest_frame
        latest_frame = frame

        amps = frame["amplitudes"]
        amp_min = min(amps) if amps else 0.0
        amp_max = max(amps) if amps else 0.0
        amp_mean = (sum(amps) / len(amps)) if amps else 0.0

        print(
            f"[{addr[0]}] seq={frame['sequence']:>8} node={frame['node_id']} "
            f"sc={frame['num_subcarriers']:>4} rssi={frame['rssi']:>4}dBm "
            f"noise={frame['noise_floor']:>4}dBm "
            f"amp(min/mean/max)={amp_min:6.1f}/{amp_mean:6.1f}/{amp_max:6.1f}"
        )

        self.loop.create_task(broadcast(frame))


async def broadcast(frame: dict):
    if not clients:
        return
    amps = frame["amplitudes"]
    payload = json.dumps(
        {
            "sequence": frame["sequence"],
            "node_id": frame["node_id"],
            "rssi": frame["rssi"],
            "noise_floor": frame["noise_floor"],
            "num_subcarriers": frame["num_subcarriers"],
            "amplitudes": amps,
            "amp_mean": (sum(amps) / len(amps)) if amps else 0.0,
        }
    )
    dead = []
    for ws in clients:
        try:
            await ws.send_str(payload)
        except ConnectionResetError:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


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
  canvas { background:#11141c; border:1px solid #232838; border-radius:8px; width:100%; height:220px; display:block; margin-bottom:1.2rem; }
  .status { font-size:.8rem; color:#8892a6; }
  .status.live { color:#5ee6a0; }
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
</div>

<div class="label" style="margin-bottom:.4rem;color:#8892a6;font-size:.75rem;text-transform:uppercase;letter-spacing:.05em;">Amplitude per subcarrier</div>
<canvas id="amp" height="220"></canvas>

<div class="label" style="margin-bottom:.4rem;color:#8892a6;font-size:.75rem;text-transform:uppercase;letter-spacing:.05em;">RSSI / mean amplitude history</div>
<canvas id="hist" height="220"></canvas>

<script>
const ampCanvas = document.getElementById('amp');
const histCanvas = document.getElementById('hist');
const statusEl = document.getElementById('status');

function fitCanvas(c) {
  const rect = c.getBoundingClientRect();
  c.width = rect.width * devicePixelRatio;
  c.height = rect.height * devicePixelRatio;
}
window.addEventListener('resize', () => { fitCanvas(ampCanvas); fitCanvas(histCanvas); });
fitCanvas(ampCanvas); fitCanvas(histCanvas);

const rssiHistory = [];
const ampHistory = [];
const HISTORY_LEN = 200;

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

  ws.onmessage = (ev) => {
    const f = JSON.parse(ev.data);
    document.getElementById('seq').textContent = f.sequence;
    document.getElementById('node').textContent = f.node_id;
    document.getElementById('sc').textContent = f.num_subcarriers;
    document.getElementById('rssi').textContent = f.rssi + ' dBm';
    document.getElementById('noise').textContent = f.noise_floor + ' dBm';

    drawAmplitudes(f.amplitudes);

    rssiHistory.push(f.rssi);
    ampHistory.push(f.amp_mean);
    if (rssiHistory.length > HISTORY_LEN) rssiHistory.shift();
    if (ampHistory.length > HISTORY_LEN) ampHistory.shift();
    drawHistory();
  };
}
connect();
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

    print(f"UDP listening on {args.udp_host}:{args.udp_port}")
    print(f"Web viewer at    http://{args.http_host if args.http_host != '0.0.0.0' else 'localhost'}:{args.http_port}")

    try:
        await asyncio.Event().wait()
    finally:
        transport.close()
        await runner.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
