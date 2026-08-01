# x-ray-wifi — System Architecture

Through-wall human localization from Wi-Fi CSI, rendered on XREAL One Pro AR
glasses.

This document is the target architecture. `CLAUDE.md` states *what* the
system is; this states *how* it is put together, what the contracts between
the pieces are, and in what order to build them. Where this document deviates
from `CLAUDE.md`, the deviation is called out explicitly with reasoning.

> **For the near-term build, read [`HACKATHON-PLAN.md`](HACKATHON-PLAN.md)
> first.** It is the subset of this architecture that is reachable in days
> with the hardware currently on hand (several ESP32s, an Arduino UNO Q, a
> drywall partition, and no XREAL-supported Unity host). The wire format,
> coordinate frames, DSP pipeline, and estimate schema defined here are used
> unchanged by that plan — it skips the learned model and the Unity client,
> not the foundations.

---

## 1. System overview

```
        PERSON SIDE                    │  WALL  │        SENSOR SIDE
                                       │        │
   ┌──────────────┐                    │        │   ┌─────────┐
   │ Illuminator  │ ~~~ 2.4 GHz ~~~~~~~│~~~~~~~~│~~▶│ node 1  │
   │  ESP32 (TX)  │  broadcast frames  │        │   ├─────────┤
   └──────────────┘  at fixed cadence  │        │──▶│ node 2  │
                                       │        │   ├─────────┤
   ┌──────────────┐                    │        │──▶│ node 3  │
   │ Overhead cam │  (training only)   │        │   ├─────────┤
   │ + ArUco tag  │                    │        │──▶│ node N  │
   └──────┬───────┘                    │        │   └────┬────┘
          │ ground-truth (x,y,t)                         │ CSI + metadata
          │                                              │ UDP, ~20–50 Hz
          │                                              ▼
          │                                   ┌──────────────────────┐
          └──────────────────────────────────▶│    Arduino UNO Q     │
             (offline, training rig only)     │   hub: ingest, DSP,  │
                                              │   fusion, inference  │
                                              └──────────┬───────────┘
                                                         │ estimate stream
                                                         │ WebSocket / JSON
                                                         ▼
                                        ┌────────────────────────────┐
                                        │ Host (Beam Pro / Galaxy    │
                                        │ S24-S25) — Unity + XREAL   │
                                        │ SDK 3.1                    │
                                        └─────────────┬──────────────┘
                                                      │ USB-C DisplayPort
                                                      ▼
                                        ┌────────────────────────────┐
                                        │   XREAL One Pro (+ Eye)    │
                                        └────────────────────────────┘
```

Five components, each independently testable:

| Component | Runs on | Language | Status |
|---|---|---|---|
| **Illuminator** | 1× ESP32 | C / ESP-IDF | Not built |
| **Sensor node** | N× ESP32-C3/S3/C6 | C / ESP-IDF | Prototype exists (`test-node/`) |
| **Hub** | Arduino UNO Q (Debian, aarch64) | Python 3.11 | Not built |
| **Training pipeline** | Workstation w/ GPU | Python 3.11 + PyTorch | Not built |
| **AR client** | Beam Pro / Galaxy S24-S25 | Unity 2022 LTS, C# | Not built |

---

## 2. Repository layout

```
x-ray-wifi/
├── firmware/
│   ├── csi-node/            promoted from test-node/ — the sensor firmware
│   └── illuminator/         broadcast-frame transmitter
├── hub/                     UNO Q runtime (Python package `xrw_hub`)
│   ├── ingest/              UDP server, wire decode, per-node ring buffers
│   ├── sync/                clock alignment, drift estimation
│   ├── dsp/                 clean → amp/phase → baseline → window
│   ├── model/               encoder + pool + head, ONNX runtime wrapper
│   ├── serve/               WebSocket estimate stream, health/debug HTTP
│   ├── record/              session recorder (raw capture to disk)
│   └── site/                site configs: node geometry, wall frame
├── training/
│   ├── capture/             overhead-camera ground-truth rig
│   ├── data/                session loading, windowing, augmentation
│   ├── models/              PyTorch definitions (shared with hub via ONNX)
│   ├── train/               training loops, evaluation, metrics
│   └── export/              PyTorch → ONNX → hub
├── ar/
│   └── XRayWifi/            Unity project (XREAL SDK 3.1)
├── tools/                   csi_viewer.py, and other dev utilities
├── docs/
│   ├── ARCHITECTURE.md      this file
│   └── decisions/           short ADR-style notes for reversals
├── adapted-node/            reference only — do not build on
└── ruview-reference-node/   reference only — do not build on
```

`test-node/build/` is compiler output and should be gitignored; it currently
accounts for the large majority of tracked files.

---

## 3. Coordinate frames

Getting this wrong is the single most likely source of silent bugs, so it is
defined once here and every component refers back to it.

**Wall frame (`W`)** — the canonical frame. Everything the hub emits is in
this frame.

- Origin: a physically marked point on the **sensor-side face of the wall**,
  at floor level. Mark it with tape. It never moves for the life of a site
  config.
- `+x`: horizontal, along the wall, to the right **as seen by an observer on
  the sensor side facing the wall**.
- `+y`: horizontal, perpendicular to the wall, pointing **away from the
  observer, into the far room**. This is "depth behind the wall" and is
  always positive for a person being tracked.
- `+z`: vertical, up.
- Right-handed. Units: meters.

**Node frame** — each node's position is a point in `W`; its antenna
orientation is a unit vector in `W`. Both live in the site config, not in
firmware.

**Camera frame** — the overhead camera is calibrated to produce a homography
from image pixels to the floor plane of `W` (`z = 0`). Calibration is stored
per capture session.

**AR frame** — Unity's world frame, left-handed and Y-up, origin wherever the
glasses initialized tracking. A single calibration step (§8.3) establishes
the rigid transform `T_AR←W`. Note the handedness flip: `W` is right-handed,
Unity is left-handed. The conversion is `(x_u, y_u, z_u) = (x_w, z_w, y_w)`.
Write it once, in one place, and unit-test it.

---

## 4. Wire and data contracts

### 4.1 Node → Hub: CSI frame (protocol v2)

Extends the existing ADR-018 20-byte header. The current header is preserved
byte-for-byte through offset 19, so the existing `tools/csi_viewer.py` keeps
working, and a new magic distinguishes v2.

| Offset | Size | Type | Field | Notes |
|---:|---:|---|---|---|
| 0 | 4 | u32 | `magic` | `0xC5110002` for v2 |
| 4 | 1 | u8 | `node_id` | **from NVS**, not compile-time |
| 5 | 1 | u8 | `num_antennas` | 1 on all current targets |
| 6 | 2 | u16 | `num_subcarriers` | `iq_len / (2 × antennas)` |
| 8 | 4 | u32 | `freq_mhz` | **derived from `rx_ctrl.channel`** |
| 12 | 4 | u32 | `sequence` | per-node, monotonic |
| 16 | 1 | i8 | `rssi` | dBm |
| 17 | 1 | i8 | `noise_floor` | dBm |
| 18 | 1 | u8 | `ppdu_type` | 0=HT/legacy, 1=HE-SU, 2=HE-MU, 3=HE-TB, 0xFF=unknown |
| 19 | 1 | u8 | `flags` | bit0 = 40 MHz |
| **20** | **8** | **u64** | **`t_node_us`** | **new** — `esp_timer_get_time()` at CSI callback |
| **28** | **1** | **u8** | **`proto_ver`** | **new** — `2` |
| **29** | **1** | **u8** | **`tx_id`** | **new** — illuminator ID that produced this frame, 0=unknown |
| **30** | **2** | **u16** | **`reserved`** | **new** — zero |
| 32 | 2×S | i8[] | payload | interleaved `(imag, real)` per subcarrier |

Byte layout for offsets 18–19 is deliberately identical to RuView's ADR-110
extension so their decoders remain a valid reference.

**`t_node_us` is the most important addition.** Without a transmit-side
timestamp, multi-node fusion has to rely on hub arrival time, which folds in
per-node queueing delay and network jitter. There is currently no timestamp
anywhere in the pipeline; nothing beyond single-node visualization is
possible until there is one.

### 4.2 Deviation from `CLAUDE.md`: sensor metadata placement

`CLAUDE.md` §1 says each measurement carries metadata including sensor
position and antenna orientation. This architecture **puts deployment
geometry in a hub-side site config keyed by `node_id`, not in the packet.**

Reasoning: position and orientation change every time you physically move a
node, which during experimentation is constantly. Putting them in firmware
means a reflash per repositioning; putting them in every packet wastes
bandwidth on a value that changes at most daily. Keeping them hub-side means
you edit one YAML file and restart one process.

The *intent* of the `CLAUDE.md` requirement is preserved in full: the fused
measurement record carries this metadata, and it is fed to the model as
input exactly as specified. The hub joins packet → metadata on `node_id` at
ingest, so downstream code sees a single enriched record. Only the physical
location of the source-of-truth differs.

Metadata that genuinely varies per-packet (RSSI, noise floor, PPDU type,
timestamp, sequence) stays on the wire.

### 4.3 Site config

`hub/site/<site-name>.yaml`:

```yaml
site: lab-partition-a
wall:
  # Wall-frame origin is implicit at (0,0,0); these describe extent.
  width_m: 3.6          # along +x
  height_m: 2.4         # along +z
  material: drywall     # drywall | wood | brick | concrete
  thickness_m: 0.12

illuminators:
  - tx_id: 1
    position_m: [1.8, 2.5, 1.0]   # person side → y > 0
    tx_rate_hz: 50
    channel: 6

nodes:
  - node_id: 1
    position_m: [0.2, -0.4, 1.1]  # sensor side → y < 0
    antenna_dir: [0.0, 1.0, 0.0]  # unit vector, facing the wall
    board: esp32-c3
  - node_id: 2
    position_m: [1.8, -0.4, 1.1]
    antenna_dir: [0.0, 1.0, 0.0]
    board: esp32-c3
  # ...

capture:
  channel: 6
  window_s: 1.5
  window_stride_s: 0.25
  target_rate_hz: 50
```

### 4.4 Hub → AR client: estimate stream

WebSocket, JSON, one message per inference tick (target 10 Hz).

```json
{
  "t_us": 1738450123456789,
  "site": "lab-partition-a",
  "present": 0.91,
  "position_m": [1.42, 1.85],
  "covariance": [[0.18, 0.03], [0.03, 0.44]],
  "height_m": null,
  "nodes_online": [1, 2, 4, 5],
  "nodes_expected": [1, 2, 3, 4, 5],
  "quality": 0.78,
  "model": "fuse-v3@a1b2c3d"
}
```

- `position_m` is `[x, y]` in the wall frame; `y > 0` is depth behind the wall.
- `covariance` is the 2×2 position covariance in m². The AR client renders
  its 95% ellipse directly. Depth uncertainty will normally dominate
  lateral — that asymmetry is real information and the ellipse should show
  it rather than being collapsed to a circle.
- `quality` folds node availability and per-node link quality into one
  number for the operator HUD. It is *not* a substitute for `covariance`.
- `model` is a version tag so a recording can be traced to the weights that
  produced it.

When `present < threshold`, the client renders nothing. The hub always sends
the message anyway — silence and "no one there" must be distinguishable.

### 4.5 Recorded session format

One directory per capture session, written by `hub/record/`:

```
sessions/2026-08-01_lab-a_walk-01/
├── session.yaml        site config snapshot + start time + operator notes
├── csi.parquet         one row per received frame, all nodes interleaved
├── gt.csv              t_us, x_m, y_m, marker_confidence
├── camera_calib.yaml   homography, intrinsics, camera pose in W
└── video.mp4           overhead footage, for debugging label quality
```

`session.yaml` embeds a **copy** of the site config rather than referencing
it. Sessions are immutable historical records; a site config edit must never
retroactively change what a past session means.

---

## 5. Firmware

### 5.1 Sensor node (`firmware/csi-node/`)

Promoted from `test-node/`. Changes needed, in priority order:

1. **Add `t_node_us`** to the header (§4.1). Blocks everything downstream.
2. **`node_id` from NVS.** Currently `static const uint8_t NODE_ID = 1;`.
   With N nodes this means N firmware builds. Adopt the provisioning
   approach from `ruview-reference-node/esp32-csi-node/provision.py`: a
   serial command writes SSID, password, hub IP, and node ID into NVS, so
   one binary flashes to every board.
3. **Derive `freq_mhz` from `rx_ctrl.channel`** instead of the hardcoded
   `2412`. The one-line switch in
   `ruview-reference-node/esp32-csi-node/main/csi_collector.c:141` is
   directly reusable. Today the field lies whenever the AP isn't on
   channel 1.
4. **Timestamp inside the CSI callback**, not in the TX worker. The queue
   between them can hold 10 frames; stamping late injects up to 200 ms of
   error.
5. **Set the channel explicitly** and disable Wi-Fi power save at boot
   (`esp_wifi_set_ps(WIFI_PS_NONE)` is already there — keep it).
6. **Report health**: a low-rate status packet with CSI callback rate, queue
   drop count, and send-failure count. Without this, a node that is up but
   producing nothing is indistinguishable from a healthy one.

Deliberately **not** ported from the RuView reference node: display/UI, mmWave
and vitals sensors, WASM runtime, OTA, mesh sync, edge processing. All out of
scope per `CLAUDE.md`.

### 5.2 Illuminator (`firmware/illuminator/`)

New, and small — a few hundred lines. Transmits broadcast frames at a fixed
rate via `esp_wifi_80211_tx()` on a fixed channel, embedding its `tx_id`.

Why this is a first-class component rather than an optimization:

- **Deterministic capture rate.** Passive listening depends on ambient
  traffic. The RuView firmware README documents yield collapsing to 0 pps
  under real conditions.
- **Comparable measurements across nodes.** When every node's CSI comes from
  the same transmission, differences between nodes are attributable to
  geometry. With nodes measuring different ambient packets from different
  transmitters, they are not comparable at all — and comparability across
  nodes is the entire signal this project depends on.
- **Controlled geometry.** You choose where the transmitter sits.

Two geometries to evaluate empirically, both supported by the config:

- **Through-transmission** (illuminator on the person side, nodes on the
  sensor side): person is between TX and RX. Strongest perturbation, best
  expected accuracy. Requires power on the far side.
- **Reflection** (illuminator on the sensor side): person's reflection
  returns through the wall twice. Weaker, but nothing needs to be placed in
  the monitored space.

Start with through-transmission to establish that the effect is measurable at
all, then measure how much reflection-mode costs you.

---

## 6. Hub (Arduino UNO Q)

### 6.1 Platform notes

The UNO Q's application processor is a Qualcomm Dragonwing QRB2210: quad-core
Cortex-A53 at 2.0 GHz with an Adreno 702 GPU, running Debian. There is no
usable NPU path today — Qualcomm's product brief cites TensorFlow Lite on CPU
and GPU. **Plan for CPU inference**, on four A53 cores. This is a real
constraint and it shapes the model budget: keep the encoder under a few
hundred thousand parameters and it will run comfortably at 10 Hz. The
STM32U585 MCU is unused by this project.

Runtime: Python 3.11 with NumPy and ONNX Runtime. Python is fast enough
because the per-window work is a handful of small NumPy operations plus one
ONNX call; if profiling later says otherwise, the DSP stage is the piece to
rewrite, not the whole hub.

### 6.2 Stage pipeline

```
UDP :5005
   │
   ▼
┌─────────────┐  decode header, validate magic/length, join site metadata
│  ingest     │  → CsiFrame{node_id, seq, t_node_us, t_hub_us, rssi,
└──────┬──────┘                noise, channel, iq[S], meta}
       ▼
┌─────────────┐  per-node clock offset estimate → t_common_us
│  sync       │  drop frames outside the alignment tolerance
└──────┬──────┘
       ▼
┌─────────────┐  drop null/guard subcarriers, reject on RSSI/length outliers,
│  clean      │  Hampel filter across time per subcarrier
└──────┬──────┘
       ▼
┌─────────────┐  amplitude = |H_k| ; phase unwrap + linear detrend
│  extract    │  → (amp[S], phase[S]) per frame
└──────┬──────┘
       ▼
┌─────────────┐  subtract per-node, per-subcarrier empty-room baseline
│  normalize  │  → (Δamp, Δphase), zero-mean in a still empty room
└──────┬──────┘
       ▼
┌─────────────┐  1.5 s windows, 0.25 s stride, resample to fixed T
│  window     │  → tensor [T, S, 2] per node
└──────┬──────┘
       ▼
┌─────────────┐  shared encoder per node → embedding[128] + conf logit
│  encode     │  masked, confidence-weighted mean pool → fused[128]
│  + fuse     │
└──────┬──────┘
       ▼
┌─────────────┐  MLP → present logit, position mean, position log-variance
│  head       │
└──────┬──────┘
       ▼
┌─────────────┐  temporal smoothing, threshold, JSON, WebSocket broadcast
│  serve      │
└─────────────┘
```

Every stage is a pure function over the previous stage's output, with a
defined dataclass at each boundary. That is what makes it possible to record
at any stage, replay offline, and unit-test in isolation — which you will
need constantly, because RF bugs are not reproducible live.

### 6.3 Clock synchronization

Windows are 1.5 s and the pipeline runs at 10 Hz, so alignment to ~10 ms is
sufficient. Achieving it:

1. Every node runs SNTP against the hub.
2. Hub stamps `t_hub_us` on arrival, tracks the per-node minimum of
   `t_hub_us − t_node_us` over a sliding window, and treats that minimum as
   the offset estimate. Taking the minimum rejects queueing delay, which is
   always additive.
3. Frames are binned into common-time buckets; a node with no frame in a
   bucket is masked out for that window.

An ESP-NOW leader/follower scheme (RuView's ADR-110, measured at ~104 µs
offset stdev) is the upgrade path if microsecond alignment ever becomes
necessary. It is not necessary now, and building it first would be a
month spent on precision the model cannot exploit.

### 6.4 Baseline capture

A hub CLI command records ~60 s of an empty, still room and writes a
per-node, per-subcarrier mean and standard deviation. Baselines are
timestamped and versioned; a baseline older than the current session's
furniture layout is a silent accuracy killer, so the hub logs baseline age
on every startup and warns past a configurable threshold.

Baselines must be captured **with the illuminator running**, since they must
characterize the same channel conditions as live operation.

---

## 7. Model and training

### 7.1 Architecture

Deliberately a starting point, not a commitment. `CLAUDE.md` is explicit that
the encoder and head are placeholders to be chosen empirically once real data
exists — this is the thing to prototype against first, and the code should
make swapping either one a config change.

**Per-node encoder** (shared weights across all nodes):

```
input   [T=48, S=52, C=2]        Δamplitude, Δphase
        ↓ 2D conv stack over (time × subcarrier), 3 blocks, BN + ReLU
        ↓ global average pool over both axes            → [128]
        ↓ concat node metadata [position(3), antenna_dir(3),
                                rssi_norm, noise_norm, yield_norm]  → [138]
        ↓ MLP 138 → 128
output  embedding [128] + confidence logit (scalar)
```

Treating the window as a time × subcarrier image is the natural first choice
because motion appears as diagonal structure across both axes simultaneously,
which a convolution captures directly. A temporal-convolution or small
transformer variant is worth benchmarking once there is data to benchmark on.

**Fusion** (permutation-invariant, cardinality-agnostic):

```
w_i    = sigmoid(conf_logit_i) × mask_i
fused  = Σ w_i · emb_i / (Σ w_i + ε)        concat  max_i(emb_i · mask_i)
                                            → [256]
```

The max term is concatenated alongside the weighted mean so that a single
node with a strong, unambiguous signal is not diluted by several nodes seeing
nothing. Both terms are permutation-invariant and both are indifferent to how
many nodes contributed, so the dropout-tolerance property is preserved.

**Head:**

```
fused [256] → MLP → { present_logit,
                      position_mean [2],
                      position_logvar [2] }
```

Diagonal covariance to start. Upgrade to a full 2×2 via Cholesky
parameterization once the diagonal model is calibrated, since depth and
lateral error are likely correlated.

**Loss:** `BCE(present)` + `𝟙[present] · GaussianNLL(position)`. The
indicator matters — regressing a position when nobody is there teaches the
model to hallucinate.

### 7.2 Training-time robustness

Directly serving the `CLAUDE.md` robustness requirement:

- **Random node dropout.** Each sample, drop each node independently with
  p ∈ [0, 0.5], always keeping ≥ 1. This is what forces the model to use the
  metadata rather than memorize node identities.
- **Layout variation.** Record sessions with physically different node
  placements and train across all of them.
- **Validity masks** everywhere, so a masked node contributes exactly zero
  and never leaks through a normalization statistic.
- **Baseline jitter.** Perturb the baseline slightly during training to
  simulate drift between calibration and use.
- **Additive noise and RSSI scaling** for link-quality variation.
- **Permutation** is free by construction — no augmentation needed, which is
  itself a good argument for this fusion design.

### 7.3 Evaluation

Report honestly and separately:

- **Presence:** precision/recall on held-out sessions, including empty-room
  sessions. An empty-room false-positive rate is mandatory; presence models
  that look excellent on occupied data often alarm constantly on empty data.
- **Position:** median and 90th-percentile Euclidean error, broken out by
  lateral vs depth. Depth will be worse. Report both.
- **Calibration:** does the predicted covariance match observed error? A
  reliability diagram. An uncertainty estimate the AR layer renders as an
  ellipse is worse than useless if it isn't calibrated.
- **Node-dropout curve:** error as a function of how many nodes are live.
- **Held-out layout:** train on layouts A and B, test on C.

Every number is tagged with the session set it came from. A number from
sessions the model trained on is not a result.

### 7.4 Deployment path

PyTorch → ONNX → ONNX Runtime on the hub. Model definitions live in
`training/models/` and are the single source of truth; the hub never
reimplements a layer. `training/export/` emits the ONNX file plus a JSON
sidecar recording input shape, normalization constants, subcarrier selection,
and the git SHA. The hub refuses to load a model whose sidecar disagrees with
the site config.

### 7.5 Capability ladder

Do not attempt continuous position first. Climb:

1. **Presence** — is anyone behind the wall? Binary, one node sufficient,
   and achievable with a threshold on windowed amplitude variance. **No
   training required.**
2. **Motion vs still** — a stationary person is dramatically harder than a
   moving one, and knowing which regime you're in is itself useful output.
   No training required.
3. **Zone classification by RF fingerprinting** — divide the far room into a
   grid, record the per-node disturbance vector at each cell, classify live
   readings by k-nearest-neighbour against that library. This is where
   multi-node fusion first proves it does something. **Needs ~15 minutes of
   collection and no camera, no GPU, and no neural network** — labels are
   "the person is standing on taped square 5." See §7.6.
4. **Continuous (x, y) regression** — the stated goal. First reachable by
   interpolating between fingerprint cells, then properly by the learned
   model of §7.1.
5. **Height / z** — only if 1–4 are solid.

Each rung is a shippable demo and a checkpoint on whether the physics is
cooperating. Reaching rung 3 confirms the whole premise; failing at rung 3
means rung 4 will not work and something upstream needs fixing.

Rungs 1–3 are all reachable without the training pipeline of §7.1–7.4. That
pipeline is what buys generalization across layouts and rooms; it is not
what buys a working demo.

### 7.6 Fingerprinting (the no-training route to rungs 3–4)

**Collection.** Tape a grid on the far-room floor — for a 3×3 m space, 9 to
16 cells is plenty. With the illuminator running and baselines captured, a
person stands on each cell for ~20 s while the hub records. Total: under 15
minutes. Repeat facing two or three different directions per cell if time
allows, since body orientation measurably changes the signature.

**Feature vector.** Per node, over a 1.5 s window, compute a small set of
scalars: mean and standard deviation of Δamplitude across subcarriers, the
per-subcarrier temporal variance summed, and mean |Δphase| after detrending.
Concatenate across nodes in fixed `node_id` order, with masked-out nodes
zero-filled. For 5 nodes × 4 features this is a 20-dimensional vector.

**Live inference.** Compute the same vector, find the *k* nearest cells
(k = 3) by Euclidean distance in a per-feature standardized space, and output
the distance-weighted centroid of their coordinates. Uncertainty comes for
free: the spread of the *k* neighbours' positions is a direct, honest
covariance estimate, and a large distance to even the nearest neighbour means
"this looks like nothing I've seen" — which is exactly the empty-room signal.

**Why this is the right hackathon choice.** It is O(minutes) to collect, has
no training loop to debug, degrades gracefully, and produces a calibrated
uncertainty naturally. Its weakness is that it does not generalize — move a
node and the entire library is void. That weakness is precisely what the
learned model of §7.1 exists to fix, which makes fingerprinting a genuine
stepping stone rather than a detour: the DSP pipeline, session recorder,
feature code, and estimate schema are all shared.

---

## 8. AR client

### 8.1 Hardware constraints (verified against XREAL SDK 3.1 docs)

| Capability | XREAL One Pro | With XREAL Eye | Air 2 Ultra |
|---|---|---|---|
| Head tracking | 3DoF | **6DoF** | 6DoF |
| Plane detection | No | **No** | Yes |
| Image tracking | No | **No** | Yes |
| Spatial anchors | No | **No** | Yes |
| Hand tracking | No | **No** | Yes |
| Raw camera access | — | **Yes** | Yes |

Additional constraints:

- **Host device must be a Beam Pro or Samsung Galaxy S24/S25.** XREAL reduced
  supported phones starting with NRSDK 2.3.0. A PC cannot run a spatial app;
  it can only drive the glasses as a display.
- **6DoF requires XREAL Eye + SDK 3.1.0 + a glasses firmware update.**
- Unity 2021.3+; SDK 3.x uses Unity XR Plugin / AR Foundation, not the
  legacy NRSDK API.
- The `ControlGlasses` (phone) or `MyGlasses` (Beam Pro) companion app must
  be installed, and "display over other apps" permission granted.

The absence of spatial anchors and image tracking is the significant one: the
conventional way to register AR content to a physical wall is unavailable and
must be built by hand.

### 8.2 Three tiers

**Tier 0 — Black-background overlay, no SDK. ← currently the committed path.**
The hub serves a browser page, displayed full-screen from any USB-C
DisplayPort source, with the glasses in their native "Anchor" mode to pin it
in space.

The reason this is not merely a fallback: XREAL glasses use **birdbath
optics, which are additive**. Black pixels emit no light and are therefore
fully transparent to the wearer. A page with a pure-black background and a
glowing marker reads as a real AR overlay, not as a floating rectangle. So
Tier 0 gets most of the *perceptual* result of Tier 1 with none of its
hardware requirements.

What it cannot do is world-lock content — the marker is fixed relative to the
virtual screen, not to the room. With the wearer standing at a marked
position facing the wall, that difference is not visible.

Design the page accordingly: pure `#000` background, no chrome, no panels, no
white text on dark gray. Anything non-black is emitted light.

Zero SDK, zero extra hardware, works today, and doubles as the primary
debugging display for all sensing work. **Build this first regardless of
which tier is the eventual target.** Everything downstream is easier to debug
when you can see what the hub thinks is happening.

**Tier 1 — 3DoF world-locked bearing.** Unity + XREAL SDK on a supported
host, glasses without Eye. 3DoF gives orientation but not position, so the
system knows which way the wearer is looking but not where they stand. This
is workable with a fixed viewing position: the wearer stands on a marked
spot facing the wall and presses a button to set the yaw reference. The
marker then renders at the correct bearing and apparent distance. It stays
correct as long as the wearer doesn't walk.

**Tier 2 — 6DoF world-locked position.** Adds XREAL Eye. The wearer can move
freely and the marker stays fixed relative to the wall. Requires solving the
`T_AR←W` alignment problem (§8.3).

### 8.3 Establishing `T_AR←W`

Two options, both implementable without the SDK features One Pro lacks.

**(a) Manual origin capture.** Mark a floor position and heading on the
sensor side, at a known offset from the wall-frame origin. The wearer stands
there facing the wall and presses a button; the app captures the current 6DoF
pose and computes `T_AR←W` from it. Trivial to build, accurate to how
precisely someone can stand on a taped X, and must be redone after any
tracking loss.

**(b) Custom fiducial tracking.** Tape an ArUco marker on the wall at a known
coordinate in `W`. The app reads the raw camera texture from XREAL Eye, runs
ArUco detection and `solvePnP` (OpenCV for Unity), and recovers the glasses'
pose in the wall frame continuously. This reimplements the image tracking the
SDK doesn't provide on this hardware.

Recommendation: **build (a) first, then (b).** (a) unblocks the AR work in an
afternoon. (b) is the real answer, and it reuses the same ArUco tooling as
the training ground-truth rig (§9) — one detection stack, two consumers.

### 8.4 Rendering

- Marker at `(x, y)` from the estimate stream, at a plausible torso height.
- **Uncertainty ellipse** on the floor plane, drawn from the covariance at
  95%. Larger and softer as confidence drops. Rendered in world space so it
  scales correctly with viewing distance.
- **Below the presence threshold, render nothing.** Not a faded marker — a
  faded marker reads as "weak detection," which is a claim. Nothing reads as
  "no claim."
- Wall plane drawn as a faint grid so the wearer understands what the marker
  is behind, and so misregistration is visible rather than silent.
- Operator HUD in a corner: node online count, estimate rate, staleness,
  tracking state. If the stream goes stale past ~1 s, gray everything out —
  a frozen marker showing a stale position is the most dangerous failure
  mode in the system.

### 8.5 Development decoupling

The Unity app connects to a WebSocket emitting the §4.4 schema. A
`tools/fake_hub.py` emits synthetic estimates — a dot on a circuit, a random
walk, dropouts, uncertainty sweeps. The AR client is developed and tested
entirely against it. No AR work is ever blocked on sensing, and vice versa.

---

## 9. Training-data capture rig

- **Camera:** overhead, wide-angle, viewing the far room. Fixed mount —
  bumping it invalidates every session recorded after.
- **Ground truth:** ArUco marker on a cap or shoulder harness. Detection via
  OpenCV; a homography maps image points to the floor plane of `W`. Marker
  height is a known constant, subtracted out.
- **Calibration:** four known floor points visible in frame define the
  homography. Stored per session.
- **Synchronization:** the capture host and hub run SNTP against the same
  source. Every session begins with a physical sync event — a hand clap
  visible on camera while someone presses a key on the hub — giving an
  independent cross-check on the software offset. Record the residual; if it
  exceeds ~50 ms, the session's labels are suspect.
- **Protocol:** empty-room baseline first, then scripted walks (perimeter,
  diagonals, dwell at grid points), then free movement, then stationary
  poses (standing, sitting, lying), then a second empty-room segment to
  detect drift.
- **Volume:** budget hours, not minutes, per node layout. This is the
  bottleneck of the whole project and the reason the recorder must be
  reliable and boring.

---

## 10. Build order

| Milestone | Deliverable | Proves |
|---|---|---|
| **M0** | Illuminator + 2 nodes w/ timestamps + NVS provisioning + hub ingest + recorder + Tier-0 radar view | End-to-end plumbing; deterministic capture rate |
| **M1** | DSP pipeline + baseline capture + offline replay; visual confirmation a person perturbs Δamp beyond the empty-room noise floor | **The physics works in your room.** Kill point if not. |
| **M2** | Ground-truth rig + first labeled multi-hour dataset across ≥ 2 node layouts | Labels are trustworthy and synchronized |
| **M3** | Presence model → zone classifier, with honest metrics incl. empty-room FPR | Multi-node fusion adds information over one node |
| **M4** | Continuous position regression + calibrated uncertainty; ONNX export; hub real-time inference at 10 Hz | The stated capability |
| **M5** | Unity AR client Tier 1, then Tier 2 with Eye + fiducial alignment | The stated experience |
| **M6** | Dropout training, held-out-layout evaluation, baseline refresh workflow | Robustness requirement |

**M1 is the go/no-go gate.** If a person walking behind the wall does not
produce a Δamplitude excursion clearly above the empty-room noise floor, no
amount of model architecture fixes it, and the response is to change the
physical setup — illuminator geometry, node placement, channel, wall — not
the code.

---

## 11. Known risks

**Room-specific models.** The model learns *this room's* multipath. Moving
furniture degrades it; a different room needs recalibration or retraining.
Cross-environment generalization is an open research problem — RuView's own
honestly-reported number is ~64% zero-shot cross-subject, recovering to 72%
only after ~30 s of in-room calibration. Plan for a calibration step in the
product, not against one.

**Single-antenna spatial resolution.** One antenna and ~52 usable
subcarriers is not much spatial information. RuView measured their own
single-ESP32 pose model at PCK@20 = 3.0% and documents distal joints as
near-random. Coarse position from several nodes is a much easier problem than
pose from one, but this is the reason node count and geometry diversity
matter more than model cleverness.

**Wall material.** Drywall and wood are fine. Brick is marginal. Concrete and
anything with a metal stud grid or foil-backed insulation will likely defeat
it. Confirm your wall early — it is a hardware fact, not a tunable.

**Stationary people.** CSI responds to *change*. A person who stops moving
becomes part of the static environment within seconds. Detecting a still
person requires the breathing-band signal, which is far weaker and is
adjacent to the vital-signs work `CLAUDE.md` scopes out. Expect motion
tracking, and be explicit that a still person may be lost.

**Data volume.** Supervised localization needs a lot of labeled data, and
collecting it is slow, physical, and tedious. This is the most likely reason
the project stalls. The recorder must be trivial to run.

**AR host dependency.** Tiers 1 and 2 require a Beam Pro or Galaxy S24/S25.
If neither is available, Tier 0 is the ceiling. Resolve this before
committing to a Unity-based plan.

---

## 12. Scope boundaries

Excluded, per `CLAUDE.md`, even though the RuView reference contains them:
DensePose and skeleton reconstruction, vital-sign monitoring, Home Assistant
and smart-home integration, vector databases and RAG, drone systems, the
RuView dashboard and product tooling, WASM edge modules, OTA, mesh
networking, and the contributor metaharness.

Borrowed from RuView, and only these: CSI acquisition technique, the ADR-018
packet framing, phase-sanitization and calibration methods, and the NVS
provisioning pattern.
