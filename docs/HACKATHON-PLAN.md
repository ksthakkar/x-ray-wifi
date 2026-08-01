# x-ray-wifi — Hackathon Plan

The subset of [`ARCHITECTURE.md`](ARCHITECTURE.md) that is reachable in days
with the hardware on hand. Same wire format, same coordinate frames, same DSP
pipeline, same estimate schema. What it drops is the learned model and the
Unity AR client — both of which are weeks of work and neither of which is
required for a working demo.

## Constraints this plan is built around

| | |
|---|---|
| Hardware | Several ESP32s, Arduino UNO Q, drywall partition |
| Not available | Beam Pro / Galaxy S24-S25, XREAL Eye, overhead camera, GPU |
| Time | Days |

Two of these are load-bearing:

**No XREAL-supported host device.** Unity + XREAL SDK requires a Beam Pro or
a Galaxy S24/S25. Neither is available, so the Unity path is closed for this
build. The replacement is the black-background overlay
([`ARCHITECTURE.md` §8.2](ARCHITECTURE.md)) driven over USB-C DisplayPort from
a laptop or from the UNO Q itself. Because birdbath optics are additive,
black pixels are transparent, so this reads as a real AR overlay rather than
a floating window.

**No camera and no time for supervised training.** Replaced by RF
fingerprinting with k-NN ([`ARCHITECTURE.md` §7.6](ARCHITECTURE.md)): tape a
numbered grid on the far-room floor, stand on each square for 20 seconds,
done in 15 minutes. Ground truth is "I am on square 5." No GPU, no training
loop, no labels to synchronize.

## What gets demoed

A person walks around behind a drywall wall. The wearer, standing on the
sensor side, sees a glowing marker drifting across their view tracking that
person's position, with an uncertainty halo that grows when the system is
unsure and vanishes entirely when the far room is empty.

---

## Day 0 — De-risk the two things that can kill the demo

Do both of these before writing pipeline code. Each is a single point of
failure that is cheap to test now and catastrophic to discover late.

**0.1 — Does the wall actually pass signal?** One node, one illuminator, on
opposite sides of the drywall. Run `tools/csi_viewer.py`. Watch the amplitude
spread with the room empty, then with someone walking behind the wall. You
need a visibly larger excursion with the person present. If there is no
difference: check for metal studs, foil-backed insulation, or an unusually
thick wall, and move to a different partition. **No amount of software fixes
a wall that doesn't pass signal.**

**0.2 — Does the UNO Q drive the glasses?** The UNO Q's USB-C port carries
DisplayPort alt-mode video output. Power the board via VIN (7–24 V) so the
USB-C port is free, connect the glasses, and confirm you get a picture. If it
works, the demo is a single self-contained board driving AR glasses, which is
a far better story. If it doesn't, use a laptop for display and keep the
UNO Q as the compute node. Decide this on day 0, not on demo morning.

---

## Day 1 — Plumbing

**Firmware.** Four changes to `test-node/`, in this order:

1. Add `t_node_us` (`esp_timer_get_time()`, captured **inside** the CSI
   callback, not in the TX worker) at header offset 20, and bump the magic to
   `0xC5110002`. Nothing multi-node works without this.
2. Read `node_id` from NVS instead of the compile-time `static const uint8_t
   NODE_ID = 1`. Otherwise you build and track N separate binaries.
   `ruview-reference-node/esp32-csi-node/provision.py` does exactly this over
   serial and is directly adaptable.
3. Derive `freq_mhz` from `rx_ctrl.channel` rather than the hardcoded `2412`
   — see `ruview-reference-node/esp32-csi-node/main/csi_collector.c:141`.
4. Pin the channel explicitly at boot so every node and the illuminator agree.

**Illuminator.** New, small firmware: `esp_wifi_80211_tx()` broadcasting a
minimal frame at 50 Hz on the fixed channel. Place it on the **person's
side** of the wall so the body sits between transmitter and receivers — the
through-transmission geometry is substantially stronger than reflection.

**Hub ingest.** Python on the UNO Q: UDP listener, header decode, per-node
ring buffer, site-config join, and a recorder that dumps raw frames to disk.
Ship a `--replay` flag from the start — you will debug against recordings far
more than against live radio, because RF bugs do not reproduce.

**End of day 1:** all nodes streaming timestamped CSI to the UNO Q, being
recorded to disk, at a steady rate you can state as a number.

---

## Day 2 — Signal

**DSP.** Clean, extract amplitude and detrended phase, subtract baseline,
window at 1.5 s with 0.25 s stride. Straight from
[`ARCHITECTURE.md` §6.2](ARCHITECTURE.md).

**Baseline capture.** A hub command that records 60 s of empty, still room —
**with the illuminator running** — and writes per-node, per-subcarrier mean
and standard deviation.

**The gate.** Plot per-node windowed Δamplitude variance over a recording of
someone walking behind the wall, versus an empty-room recording. The occupied
trace must sit clearly above the empty one. This is the same check as day 0.2
but quantitative and per-node, and it tells you which nodes are earning their
place.

If a node shows no separation, move it. Node placement matters more than
anything you will do in software this week. Spread them along the wall rather
than clustering, and keep them close to it.

**Presence detection.** A threshold on that variance, with hysteresis and a
few-frame debounce. This is rung 1 of the capability ladder and it is a
working demo on its own.

**End of day 2:** the hub prints "someone is behind the wall" correctly, and
you have plots proving the physics works.

---

## Day 3 — Localization

**Grid.** Tape numbered squares on the far-room floor. 9 to 16 cells for a
3×3 m space. Record the cell coordinates in the site config.

**Collect.** Illuminator running, baselines fresh. A person stands on each
cell for 20 s while the hub records, labeled by cell number. Add two or three
body orientations per cell if time allows — orientation changes the signature
measurably. Then record 60 s of empty room as the null class. Under 20
minutes total.

**Feature vector.** Per node, per 1.5 s window: mean and standard deviation of
Δamplitude across subcarriers, summed per-subcarrier temporal variance, and
mean |Δphase| after detrending. Concatenate across nodes in fixed `node_id`
order, zero-filling masked nodes. Five nodes gives a 20-dimensional vector.

**k-NN.** Standardize per feature. At inference, take the 3 nearest cells and
output their distance-weighted centroid. The spread of those 3 neighbours is
your covariance estimate — honest, free, and calibrated by construction. A
large distance to even the nearest neighbour means "this matches nothing I
recorded," which is your empty-room signal and a genuine out-of-distribution
guard.

**Smoothing.** A light temporal filter on the output position. Raw per-window
estimates will jitter; a person cannot teleport, and the demo reads far
better with 0.5 s of smoothing. Do not over-smooth — lag is more noticeable
than jitter.

**End of day 3:** the hub emits the [`ARCHITECTURE.md` §4.4](ARCHITECTURE.md)
estimate JSON over WebSocket, and the position tracks a walking person.

---

## Day 4 — Display

Serve a single full-screen page from the hub. Design rules, which matter more
than they sound:

- **Pure `#000` background.** Every non-black pixel is emitted light. No
  panels, no cards, no dark-gray surfaces, no borders.
- **The marker**: a soft glowing blob, not a hard dot. Hard edges read as
  false precision, and false precision is the one thing this display must
  never claim.
- **The uncertainty ellipse**: drawn from the covariance at 95%, in the same
  perspective as the marker. Depth uncertainty will exceed lateral — let the
  ellipse show that rather than rounding it to a circle.
- **Below the presence threshold, render nothing at all.** Not a dimmed
  marker. A dimmed marker reads as "weak detection," which is a claim.
  Nothing reads as "no claim."
- **The wall**: a faint grid line so the wearer understands what the marker is
  behind. This also makes misregistration visible rather than silent.
- **Staleness**: if the estimate stream goes quiet for more than a second,
  clear the display. A frozen marker showing a stale position is the worst
  possible failure, because it looks exactly like a working system.

Two views worth having, toggled by keypress: a **first-person** view (marker
positioned as if seen through the wall, for the demo) and a **top-down radar**
(wall as a line, nodes as dots, person as a marker — for debugging and for
explaining the system to onlookers).

Build against `tools/fake_hub.py` emitting synthetic estimates so display
work never blocks on sensing.

**End of day 4:** wearer puts on the glasses, someone walks behind the wall,
the marker follows.

---

## Day 5 — Harden

- **Recover from node loss.** Unplug a node mid-demo; the system should
  degrade, not crash. The mask path makes this nearly free, but test it.
- **Re-baseline procedure.** A single command, because the room the demo
  happens in will not be the room you calibrated in.
- **Numbers for the pitch.** Median position error against held-out grid
  standings, empty-room false-positive rate, and end-to-end latency. Measure
  them; do not estimate them. A demo with three honest numbers is far more
  convincing than one with none.
- **Rehearse the failure.** Know what you say when the marker jumps, because
  it will. "The uncertainty halo just grew, which is the system telling you
  it lost confidence" is a much better line than silence.

---

## Cut list, in order

If time runs short, drop in this order:

1. Multiple body orientations per grid cell → one orientation only.
2. The first-person view → top-down radar only. Less impressive, equally
   functional, and easier to explain.
3. Continuous position → zone highlighting. "Person is in the top-left
   quadrant" is a real, honest result and much more robust than a moving dot.
4. The UNO Q → run the hub on a laptop. Same Python, zero code change. Say
   so out loud; portability is a feature.
5. Grid fingerprinting entirely → presence detection only. "Someone is behind
   this wall, and the display goes dark when they leave" still demonstrates
   the core physics, and it works with a single node.

Rung 1 (presence) is achievable on day 2 with one node. Everything after that
is upside. Protect the day-2 result.

---

## What this plan deliberately does not build

Deferred to [`ARCHITECTURE.md`](ARCHITECTURE.md), not abandoned:

- The learned per-node encoder and fusion model (§7.1–7.4). Fingerprinting
  does not generalize — move a node and the library is void. The learned
  model is what fixes that, and it needs a camera rig, hours of labeled
  capture, and a GPU.
- The overhead-camera ground-truth rig (§9).
- The Unity AR client, Tiers 1 and 2 (§8.2–8.3). Blocked on a Beam Pro or
  Galaxy S24/S25 regardless of time.
- ESP-NOW microsecond clock sync (§6.3). SNTP plus hub-side offset estimation
  is well inside what 1.5 s windows need.

Everything built in this plan — wire format, site config, DSP pipeline,
session recorder, feature extraction, estimate schema, display — is used
unchanged by the full architecture. None of it is throwaway.
