"""
Live multi-node CSI dashboard — one panel per receiver.

Built for the transmitter/receiver setup: one ESP32 broadcasts, several receive,
and this shows each receiver's channel response side by side so you can walk
between a TX/RX pair and watch that link react while the others do not.

That side-by-side view is the point. A single-node display cannot distinguish
"a body moved" from "the environment drifted", because both look like change.
Several nodes seeing DIFFERENT changes at the same moment is geometric evidence.

Displayed per node, all computed live (nothing is stored -- use csi_record.py
for that):

  * amplitude per subcarrier, with a faint frozen reference for comparison
  * deviation from that reference (L1 distance): the headline "something changed"
  * a history trace of the deviation
  * frame rate, RSSI, and TX-beacon delivery ratio

Press the "freeze reference" button with the room empty; every panel then shows
deviation from that baseline.

    python csi_live.py                 # listens on 0.0.0.0:5005, serves :8080
    python csi_live.py --http-port 9000
"""

import argparse
import asyncio
import json
import math
import socket
import struct
import time

from aiohttp import web

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

BROADCAST_HZ = 12          # display refresh; decoupled from the packet rate
SMOOTH_ALPHA = 0.3         # light smoothing of the per-node amplitude profile

clients = set()
nodes = {}                 # node_id -> NodeState


class NodeState:
    """Per-receiver live state. One instance per node_id seen."""

    def __init__(self, nid):
        self.nid = nid
        self.amps = None          # smoothed amplitude profile
        self.reference = None     # frozen baseline profile
        self.rssi = 0
        self.n_sc = 0
        self.frames = 0
        self.prev_frames = 0
        self.rate = 0.0
        self.last_seen = 0.0
        self.prev_seq = None
        self.lost = 0
        self.mac = ""
        self.deviation = 0.0
        self.dev_history = []

    def update(self, hdr, csi):
        # ESP-IDF packs each subcarrier as an (imag, real) int8 pair.
        start = 2 if hdr["first_word_invalid"] else 0
        amps = []
        for i in range(start, len(csi) // 2):
            a, b = csi[2 * i], csi[2 * i + 1]
            im = a - 256 if a > 127 else a
            re = b - 256 if b > 127 else b
            amps.append(math.hypot(re, im))
        if not amps:
            return

        # Subcarrier count can change with frame type; a profile of one width
        # cannot be compared against another, so restart when it changes.
        if self.amps is None or len(self.amps) != len(amps):
            self.amps = list(amps)
            self.reference = None
        else:
            for i, v in enumerate(amps):
                self.amps[i] += SMOOTH_ALPHA * (v - self.amps[i])

        self.n_sc = len(amps)
        self.rssi = hdr["rssi"]
        self.frames += 1
        self.last_seen = time.time()
        self.mac = hdr["mac"].hex()

        seq = hdr["sequence"]
        if self.prev_seq is not None and seq > self.prev_seq + 1:
            self.lost += seq - self.prev_seq - 1
        self.prev_seq = seq

        if self.reference:
            n = min(len(self.reference), len(self.amps))
            self.deviation = sum(abs(self.amps[i] - self.reference[i])
                                 for i in range(n)) / n
        else:
            self.deviation = 0.0

        self.dev_history.append(self.deviation)
        if len(self.dev_history) > 240:
            self.dev_history.pop(0)

    def freeze(self):
        if self.amps:
            self.reference = list(self.amps)

    def payload(self):
        return {
            "node_id": self.nid,
            "amps": [round(v, 2) for v in (self.amps or [])],
            "reference": [round(v, 2) for v in (self.reference or [])],
            "has_ref": self.reference is not None,
            "rssi": self.rssi,
            "n_sc": self.n_sc,
            "frames": self.frames,
            "rate": round(self.rate, 1),
            "lost": self.lost,
            "mac": self.mac,
            "deviation": round(self.deviation, 3),
            "dev_history": [round(v, 3) for v in self.dev_history],
            "stale": (time.time() - self.last_seen) > 3.0,
        }


class CSIProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        if len(data) < HEADER_SIZE:
            return
        vals = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        if vals[0] != MAGIC:
            return
        hdr = dict(zip(HEADER_FIELDS, vals))
        if hdr["rx_state"]:
            return
        nid = hdr["node_id"]
        if nid not in nodes:
            nodes[nid] = NodeState(nid)
            print(f"[+] node {nid} appeared ({addr[0]})")
        nodes[nid].update(hdr, data[HEADER_SIZE:])


async def rate_loop():
    """Recompute per-node frame rate once a second."""
    while True:
        await asyncio.sleep(1.0)
        for ns in nodes.values():
            ns.rate = ns.frames - ns.prev_frames
            ns.prev_frames = ns.frames


async def broadcast_loop():
    """Push state at a fixed display rate, not per packet."""
    while True:
        await asyncio.sleep(1.0 / BROADCAST_HZ)
        if not clients or not nodes:
            continue
        msg = json.dumps({"nodes": [nodes[k].payload() for k in sorted(nodes)]})
        dead = []
        for ws in clients:
            try:
                await ws.send_str(msg)
            except (ConnectionResetError, ConnectionError):
                dead.append(ws)
        for ws in dead:
            clients.discard(ws)


async def index(_req):
    return web.Response(text=INDEX_HTML, content_type="text/html")


async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    clients.add(ws)
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                if msg.data == "freeze":
                    for ns in nodes.values():
                        ns.freeze()
                    print("[*] froze reference on all nodes")
                elif msg.data == "clear":
                    for ns in nodes.values():
                        ns.reference = None
                        ns.dev_history.clear()
                    print("[*] cleared references")
    finally:
        clients.discard(ws)
    return ws


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "localhost"
    finally:
        s.close()


INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>CSI Live - multi-node</title>
<style>
 body{background:#0b0d12;color:#e6e6e6;font-family:ui-monospace,Consolas,monospace;
      margin:0;padding:1rem}
 h1{font-size:1rem;color:#9fd3ff;margin:0 0 .6rem}
 .bar{display:flex;gap:.6rem;align-items:center;margin-bottom:.8rem;flex-wrap:wrap}
 button{background:#1d2434;color:#e6e6e6;border:1px solid #2f3850;border-radius:6px;
        padding:.45rem .9rem;font-family:inherit;font-size:.8rem;cursor:pointer}
 button:hover{background:#27304a}
 .hint{font-size:.75rem;color:#8892a6}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:.8rem}
 .node{background:#151923;border:1px solid #232838;border-radius:8px;padding:.7rem}
 .node.stale{opacity:.4;border-color:#5a3a3a}
 .node.alert{border-color:#ffb765}
 .nh{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:.4rem}
 .nid{font-size:.95rem;color:#9fd3ff}
 .meta{font-size:.7rem;color:#8892a6}
 .dev{font-size:1.5rem;margin:.2rem 0}
 .node.alert .dev{color:#ffb765}
 canvas{display:block;width:100%;background:#11141c;border-radius:5px;margin-top:.35rem}
 .lbl{font-size:.65rem;color:#6b7488;text-transform:uppercase;letter-spacing:.04em;
      margin-top:.45rem}
</style></head><body>
<h1>x-ray-wifi &mdash; live multi-node CSI</h1>
<div class="bar">
  <button onclick="send('freeze')">Freeze reference (empty room)</button>
  <button onclick="send('clear')">Clear</button>
  <span class="hint" id="status">connecting...</span>
  <span class="hint">Freeze with the room empty, then walk between TX and a
    receiver &mdash; that link's deviation should rise while others stay low.</span>
</div>
<div class="grid" id="grid"></div>
<script>
const grid=document.getElementById('grid'), statusEl=document.getElementById('status');
let ws, pending=null;
const els={};

function send(m){ if(ws&&ws.readyState===1) ws.send(m); }

function ensure(nid){
  if(els[nid]) return els[nid];
  const d=document.createElement('div'); d.className='node';
  d.innerHTML=`<div class="nh"><span class="nid">node ${nid}</span>
      <span class="meta" id="m${nid}"></span></div>
    <div class="dev" id="d${nid}">--</div>
    <div class="lbl">amplitude per subcarrier (grey = frozen reference)</div>
    <canvas id="a${nid}" height="90"></canvas>
    <div class="lbl">deviation history</div>
    <canvas id="h${nid}" height="60"></canvas>`;
  grid.appendChild(d);
  els[nid]={root:d, meta:d.querySelector('#m'+nid), dev:d.querySelector('#d'+nid),
            amp:d.querySelector('#a'+nid), hist:d.querySelector('#h'+nid)};
  return els[nid];
}

function fit(c){const r=c.getBoundingClientRect();
  if(c.width!==r.width*devicePixelRatio){c.width=r.width*devicePixelRatio;
  c.height=parseInt(c.getAttribute('height'))*devicePixelRatio;}}

function drawAmps(c,amps,ref){
  fit(c); const x=c.getContext('2d'),w=c.width,h=c.height;
  x.clearRect(0,0,w,h); if(!amps.length) return;
  const mx=Math.max(...amps,...(ref||[]),1), bw=w/amps.length;
  if(ref&&ref.length){x.fillStyle='#39404f';
    for(let i=0;i<ref.length;i++){const bh=ref[i]/mx*(h-4);
      x.fillRect(i*bw,h-bh,Math.max(bw-1,1),bh);}}
  for(let i=0;i<amps.length;i++){const bh=amps[i]/mx*(h-4);
    x.fillStyle=`hsl(${200-amps[i]/mx*120},75%,58%)`;
    x.fillRect(i*bw+bw*0.25,h-bh,Math.max(bw*0.5,1),bh);}
}

function drawHist(c,hist){
  fit(c); const x=c.getContext('2d'),w=c.width,h=c.height;
  x.clearRect(0,0,w,h); if(hist.length<2) return;
  const mx=Math.max(...hist,1), sx=w/239;
  x.beginPath(); x.moveTo((239-hist.length+1)*sx,h);
  hist.forEach((v,i)=>x.lineTo((239-hist.length+1+i)*sx,h-v/mx*(h-4)));
  x.lineTo((239)*sx,h); x.closePath();
  x.fillStyle='rgba(94,230,160,0.14)'; x.fill();
  x.strokeStyle='#5ee6a0'; x.lineWidth=1.6*devicePixelRatio; x.beginPath();
  hist.forEach((v,i)=>{const px=(239-hist.length+1+i)*sx,py=h-v/mx*(h-4);
    i?x.lineTo(px,py):x.moveTo(px,py);}); x.stroke();
  x.fillStyle='#6b7488'; x.font=`${9*devicePixelRatio}px monospace`;
  x.fillText(mx.toFixed(2),3*devicePixelRatio,11*devicePixelRatio);
}

function render(){
  requestAnimationFrame(render);
  if(!pending) return; const data=pending; pending=null;
  for(const n of data.nodes){
    const e=ensure(n.node_id);
    e.root.classList.toggle('stale',n.stale);
    // Highlight when this link has clearly changed relative to its own history.
    const hmax=Math.max(...(n.dev_history.length?n.dev_history:[0]),0.001);
    e.root.classList.toggle('alert', n.has_ref && n.deviation > 0.5*hmax && hmax>0.05);
    e.meta.textContent=`${n.rate}/s  ${n.rssi}dBm  ${n.n_sc}sc  lost ${n.lost}`;
    e.dev.textContent = n.has_ref ? n.deviation.toFixed(2) : 'no ref';
    drawAmps(e.amp,n.amps,n.reference);
    drawHist(e.hist,n.dev_history);
  }
}

function connect(){
  ws=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');
  ws.onopen=()=>{statusEl.textContent='live';};
  ws.onclose=()=>{statusEl.textContent='disconnected, retrying...';setTimeout(connect,1000);};
  ws.onmessage=e=>{pending=JSON.parse(e.data);};
}
connect(); render();
</script></body></html>
"""


async def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--udp-host", default="0.0.0.0")
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8080)
    args = ap.parse_args()

    loop = asyncio.get_running_loop()
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, args.http_host, args.http_port).start()

    transport, _ = await loop.create_datagram_endpoint(
        CSIProtocol, local_addr=(args.udp_host, args.udp_port))

    tasks = [asyncio.create_task(broadcast_loop()), asyncio.create_task(rate_loop())]
    print(f"UDP  listening on {args.udp_host}:{args.udp_port}")
    print(f"View at http://{lan_ip()}:{args.http_port}")
    print("Point every receiver's CSI_TARGET_IP at that address.")
    try:
        await asyncio.Event().wait()
    finally:
        for t in tasks:
            t.cancel()
        transport.close()
        await runner.cleanup()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
