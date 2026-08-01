# x-ray-wifi

Through-wall human localization using Wi-Fi Channel State Information (CSI).
Distributed ESP32 nodes sense how a person's body distorts the ambient
multipath field on the far side of a wall; a central Arduino UNO Q fuses their
CSI streams, runs inference, and pushes a position estimate to XREAL One Pro
AR glasses so the wearer sees a marker for the person "through" the wall.

This file describes both what exists in this repo today and the system the
project is building toward, so work can be evaluated against the end goal
rather than just the current code.

## Repository layout

| Path | What it is |
|---|---|
| `test-node/` | **Active work.** PlatformIO/ESP-IDF project for a single ESP32 CSI sender — this is the real firmware base going forward. Minimal promiscuous-mode CSI capture -> UDP sender, currently pointed at a RuView `sensing-server` Docker container on a Raspberry Pi, not yet at the UNO Q. |
| `ruview-reference-node/esp32-csi-node/` | Reference only. Fuller ESP-IDF firmware pulled from RuView (CSI collector, ADR-018 packet framing, mmWave/vitals sensors, display UI, mesh sync, OTA, WASM edge runtime, etc.). Not a build base — mine it for CSI acquisition, packet framing, and calibration ideas only. |
| `ruview/` | Reference only. Full upstream RuView repository (Rust `v2/` workspace, Python `archive/v1/`, dashboard, docs, harness tooling, Home Assistant/HuggingFace/vector-db integrations, drone/vitals examples, etc.). Do not port wholesale or build on top of it. Only borrow: CSI acquisition, packet formats/framing, calibration and phase-processing techniques, and multi-sensor fusion approaches. |

No UNO Q processing code, training pipeline, or AR client exists in this repo
yet — that is the bulk of the remaining work (see below). No directory layout
is proposed for them yet; decide that when the work starts.

## End goal: system architecture

```
[Wall]
  Person side                          Sensor side
                                    +------------------+
                                    | ESP32-C3 / S3 x N|  <- CSI capture nodes
                                    | (varied position, |     around one side
                                    |  antenna orient.) |     of the wall
                                    +--------+---------+
                                             | Wi-Fi CSI streams (UDP),
                                             | tagged with sensor metadata
                                             v
                                    +------------------+
                                    |  Arduino UNO Q    |  <- central fusion +
                                    |  (4 GB RAM)       |     inference node
                                    +--------+---------+
                                             | position + confidence
                                             v
                                    +------------------+
                                    | XREAL One Pro AR  |  <- overlay marker /
                                    | glasses            |     silhouette /
                                    +------------------+     uncertainty region
```

### 1. Sensor nodes (ESP32-C3 / ESP32-S3)

- Multiple nodes positioned around one side of the wall, each receiving
  controlled Wi-Fi packets and extracting raw CSI (per-subcarrier I/Q).
- Each measurement carries metadata: sensor position, sensor type, antenna
  orientation, RSSI, packet/link quality, timestamp, and transmitter geometry.
- Nodes stream CSI + metadata to the UNO Q (current `test-node` prototype uses
  a simple binary header over UDP — a good starting point, needs a
  destination/target update from "Raspberry Pi" to the UNO Q and a metadata
  field audit against the list above).

### 2. Central processing (Arduino UNO Q, 4 GB RAM)

Owns the full pipeline from raw samples to a fused estimate:

1. **Collection** — ingest raw CSI from all connected ESP32 nodes.
2. **Cleaning** — drop malformed packets and invalid/unreliable subcarriers.
3. **Amplitude/phase extraction** — amplitude always; phase sanitization
   (unwrapping, offset removal) where usable.
4. **Baseline normalization** — calibrate against an empty-environment
   baseline per sensor/subcarrier.
5. **Windowing + feature extraction** — temporal windows -> feature vectors
   (or a raw time x subcarrier tensor, if the encoder is convolutional).
6. **Per-node encoding + fusion** — current best design (not final, but the
   working target): a shared, trained per-node encoder consumes one node's
   windowed CSI plus its metadata (position, orientation, room layout,
   RSSI/link quality) and outputs a fixed-size embedding. Embeddings from
   all currently-available nodes are then combined with a permutation-
   invariant, cardinality-agnostic pool — e.g. a masked/confidence-weighted
   average — into one fused vector. This is deliberately not RuView's
   pattern (full per-node task model -> late-fuse the *output*
   distributions across nodes, e.g. `cog-person-count`'s confidence-
   weighted log-sum): fusing at the embedding level lets ambiguous partial
   evidence from multiple nodes combine before any decision is made,
   instead of each node needing to solve the task alone. It also gets
   variable node count and dropout tolerance for free — a missing node is
   just absent from the pool, no architecture change needed — because the
   encoder is shared across nodes and the pool doesn't care how many inputs
   it sees. The pooling step can stay simple (a weighted average) precisely
   because the encoder already did the hard work of projecting each node's
   raw signal into a comparable representation.
7. **Inference** — a general head (MLP, small CNN, whatever performs best —
   not fixed) consumes the fused vector and outputs presence probability,
   position, and confidence. As with the encoder, architecture here is a
   placeholder to be chosen empirically once real captured data exists to
   prototype against — e.g. a CNN/temporal-convolution treatment of a
   per-node time x subcarrier "image" is also worth trying, either as the
   per-node encoder or as the head.
8. **Sync + comms** — timestamp alignment across nodes on the way in;
   packaging and transmission of results to the AR glasses on the way out.

Also handles synchronization and calibration bookkeeping (per-node clock
offsets, baseline capture/refresh workflow).

### 3. Model output

Given fused CSI + metadata, the model produces:

- Probability that at least one person is present.
- Estimated position in a wall-relative coordinate system: horizontal
  position and depth behind the wall, optionally height.
- Confidence/uncertainty for that position (e.g. std devs or a covariance
  matrix).

**Robustness requirement:** the system must tolerate varying numbers and
placements of ESP32 nodes. Training should use sensor validity masks, sensor
metadata as model input, and simulated receiver dropout, so the model doesn't
overfit to one fixed sensor layout.

### 4. Training data collection

- Overhead camera + ArUco markers provide timestamped ground-truth person
  position.
- Camera and CSI streams are synchronized and cut into labeled temporal
  windows for supervised training of the model, whatever its architecture
  ends up being.

### 5. AR output (XREAL One Pro)

- UNO Q transmits the final position + confidence to the glasses.
- Glasses render the estimate as a marker, silhouette, or uncertainty region
  in the space behind the wall.
- Low-confidence detections get a larger/softer uncertainty region, or are
  hidden below some confidence threshold — never shown as false precision.

## Scope boundaries (what NOT to build here)

Excluded even though present in the `ruview` reference repo: DensePose/
skeleton reconstruction, vital-sign (breathing/heart rate) monitoring, Home
Assistant integration, vector database / RAG infra, drone systems, the RuView
dashboard/product tooling, and its contributor metaharness. If a task starts
pulling in one of these, it's likely scope creep from the reference repo
rather than something this project needs.
