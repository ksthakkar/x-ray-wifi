# x-ray-wifi

Through-wall human localization using Wi-Fi CSI. See [`CLAUDE.md`](CLAUDE.md)
for the full project architecture and end goal.

This README covers the part that exists and runs today: flashing the
`test-node` ESP32 firmware and watching its CSI stream live on your laptop.

## What you need

- An ESP32 board — ESP32, ESP32-S3, or ESP32-C3 are all supported (see
  "Identifying which board you have" below).
- [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).
- Python 3.10+ on the laptop that will receive the CSI stream.
- Your laptop and the ESP32 on the same Wi-Fi network.

## Identifying which board you have

`test-node/platformio.ini` defines one PlatformIO environment per board
family. Pick the one matching your hardware and use its name (`<ENV>` below)
in every `pio` command:

| Your board | PlatformIO env (`<ENV>`) | Notes |
|---|---|---|
| ESP32-C3 (e.g. DevKitM-1) | `esp32-c3-devkitm-1` | Single-core RISC-V. |
| ESP32-S3 (e.g. DevKitC-1) | `esp32-s3-devkitc-1` | Dual-core, native USB. |
| Original ESP32 (e.g. DevKitC / esp32dev-style boards) | `esp32dev` | Dual-core, usually needs a USB-serial bridge chip. |

How to tell which one you have:

- **Check the chip silkscreen.** The main chip on the board is usually
  printed with its exact model, e.g. `ESP32-C3`, `ESP32-S3`, or plain
  `ESP32`.
- **Check the USB device.** Run `pio device list` with the board plugged in
  and look at the `Hardware ID` for your board's COM/serial port:
  - `VID:PID=303A:xxxx` → Espressif's own native-USB vendor ID. This almost
    always means an **S3 or C3** (they have USB built into the chip).
  - A different vendor ID (commonly `10C4` for CP2102 or `1A86` for CH340)
    → a USB-to-serial bridge chip, which usually means the **original
    ESP32** (no native USB).
  - Ignore any `Standard Serial over Bluetooth link` entries — those are
    virtual ports from your OS's Bluetooth stack, not the board.

Example: on this project, `pio device list` showed:

```
COM9
----
Hardware ID: USB VID:PID=303A:1001 SER=34:B7:DA:F6:42:54 LOCATION=1-6:x.0
Description: USB Serial Device (COM9)
```

`303A` narrowed it to an S3 or C3; the board's silkscreen confirmed **C3**,
so the commands below use `<ENV>=esp32-c3-devkitm-1` and `<PORT>=COM9`.

## 1. Configure credentials

The firmware reads Wi-Fi credentials and the UDP destination from a local,
gitignored header instead of hardcoded values.

```bash
cp test-node/src/credentials.h.example test-node/src/credentials.h
```

Edit `test-node/src/credentials.h`:

```c
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASS "your-wifi-password"

#define CSI_TARGET_IP "192.168.1.100"   // your laptop's LAN IP
#define CSI_TARGET_PORT 5005
```

Find your laptop's LAN IP with `ipconfig` (Windows) and make sure it's the
address on the same network the ESP32 will join, not a VPN/virtual adapter.

`credentials.h` is listed in `test-node/.gitignore` and will never be
committed — only `credentials.h.example` is tracked.

## Recommended order

Start the Python viewer (step 3) **before** powering on or resetting the
ESP32. UDP doesn't require a listener to be "connected" first, but starting
the viewer first avoids a couple of practical gotchas:
- On first run, Windows Firewall will prompt to allow `python.exe` on
  private networks — if that prompt is still pending when the ESP32 starts
  sending, those first packets get silently dropped.
- It's simply easier to confirm things are working when the receiving end
  is already up and printing, rather than wondering whether a blank viewer
  means the firmware failed or the listener isn't running yet.

So: `python tools/csi_viewer.py` first, then flash/reset the board.

## 2. Build and flash the firmware

All `pio` commands must be run from inside `test-node/` (that's where
`platformio.ini` lives). Substitute `<ENV>` and `<PORT>` from the board
identification step above:

```bash
cd test-node

pio device list                                    # find the COM port
pio run -e <ENV>                                    # build
pio run -e <ENV> -t upload --upload-port <PORT>      # flash
pio device monitor -b 115200 -p <PORT>               # optional: watch ESP32 boot/log output live
```

Worked example for a C3 on `COM9`:

```bash
cd test-node

pio device list
pio run -e esp32-c3-devkitm-1
pio run -e esp32-c3-devkitm-1 -t upload --upload-port COM9
pio device monitor -b 115200 -p COM9
```

`pio device monitor` opens a serial terminal to the board over USB so you can
see its `ESP_LOGI` output as it runs — Wi-Fi connecting, then
`Streaming CSI -> <ip>:<port>` once it's associated and sending. It's the
easiest way to confirm the board booted correctly rather than failing
silently. Exit with `Ctrl+]`.

On boot the firmware joins Wi-Fi, enables promiscuous-mode CSI capture, and
starts streaming CSI frames as UDP packets to `CSI_TARGET_IP:CSI_TARGET_PORT`
at up to ~50 Hz.

**Note on the EN/reset button (C3/S3 native-USB boards):** pressing EN drops
and re-enumerates the USB connection itself (native USB lives on the chip,
not a separate bridge chip), which can leave an already-open
`pio device monitor` session attached to a dead handle — it'll just go
silent and never show the reboot log. If you press reset and nothing
prints, fully quit the monitor (`Ctrl+C`) and reopen it fresh rather than
assuming the firmware crashed.

## 3. Run the CSI viewer on your laptop

From the repo root:

```bash
pip install -r tools/requirements.txt
python tools/csi_viewer.py
```

This starts:
- a UDP listener on `:5005` that decodes each packet's header and CSI
  amplitude/phase, printing a summary line per packet to the console, and
- a live web dashboard at `http://localhost:8080` showing a per-subcarrier
  amplitude bar chart and a scrolling RSSI/amplitude history graph.

Custom ports: `python tools/csi_viewer.py --udp-port 5005 --http-port 8080`.

## Interpreting what you see

Raw per-frame CSI amplitude looks noisy even in an empty, static room —
that's expected. Multipath is sensitive to small ambient changes, and a
single instantaneous frame isn't the right thing to eyeball; what matters
for sensing is how the amplitude *spread/variance* changes over a time
window, not any one frame's shape. This is why the eventual pipeline
(`CLAUDE.md`) has dedicated baseline-normalization and windowing stages
rather than acting on raw frames.

A quick manual sanity check that a person perturbs the field at all:

1. Let the viewer run untouched for ~30 seconds with the room otherwise
   empty/still — this is your rough baseline noise floor.
2. Walk around a few feet from the node (don't touch or block the antenna —
   that's near-field occlusion, a much bigger and less relevant effect than
   what we actually care about) and watch whether the amplitude spread
   visibly increases versus the baseline.
3. Try both moving and standing still — CSI is far more sensitive to motion
   (actively changing multipath) than to a stationary person, who after a
   moment just becomes a fixed part of the environment.

Directly covering the antenna with your hand causes an obvious RSSI drop —
that's real, but it's testing something different (physical signal
occlusion) than the through-wall multipath-perturbation effect this project
is built around, so don't read too much into it either way.

## Troubleshooting

- **Nothing shows up in the viewer:** confirm `CSI_TARGET_IP` in
  `credentials.h` matches your laptop's current IP, and that both devices
  are on the same subnet (not one on Wi-Fi and one on a different VLAN/guest
  network). Check the serial monitor for the "Streaming CSI ->" log line to
  confirm the ESP32 actually associated and started sending.
- **Firewall blocking UDP:** allow inbound UDP on the port `csi_viewer.py`
  listens on (default `5005`) through your laptop's firewall.
- **Build fails referencing `credentials.h`:** you skipped step 1 — copy
  `credentials.h.example` to `credentials.h` first.
