# Fixed-view estimate display

This service is the shared boundary between localization and rendering. It
accepts the estimate contract from `docs/ARCHITECTURE.md` §4.4, validates it,
and broadcasts the latest valid estimate to browser or Unity clients.

## Run with synthetic data

From the repository root:

```powershell
pip install -r tools/requirements.txt
python tools/fake_hub.py --scenario all
```

Open `http://localhost:8080`. The `all` scenario cycles through motion,
absence, growing uncertainty, a two-second stream pause, a forced WebSocket
reconnect, and rejection of a malformed estimate.

The production server can run independently:

```powershell
python -m hub.serve.server --config hub/site/lab-partition-a.json
```

Publish estimates from another hub stage with `POST /api/estimates`. Display
clients connect to `GET /ws/estimates`; diagnostics are at `GET /healthz`.

## Display controls

- `F`: request browser fullscreen
- `H`: toggle stream diagnostics
- `C`: enter or leave calibration
- Calibration arrows: adjust yaw and pitch
- Calibration `+` / `-`: adjust effective horizontal field of view
- Calibration `R`: reset browser-local adjustments

The site config contains the measured viewer position. Browser calibration
only corrects display alignment and is stored in local storage; it never
changes wall-frame estimates.

## One Pro setup and hardware check

1. Power the UNO Q separately so its USB-C DisplayPort output is available.
2. Open the page fullscreen at the display's active resolution.
3. Connect the One Pro, select Anchor mode, and stand on the marked viewer
   position facing the wall.
4. Long-press the X button to recenter, press `C`, and align the grid with
   measured wall edges.
5. Run `--scenario motion` and rotate your head through the intended demo
   range. Record the angle at which the wall or marker leaves the display.
6. Run `--scenario stale`; the marker must disappear within one second of the
   stream pause.
7. Run `--scenario all`; verify automatic reconnect and that no frozen marker
   remains during disconnect.
8. Measure end-to-end latency with a timestamped visual event or high-speed
   video. Repeat after ten minutes and record any Anchor drift and the
   recenter procedure.

This test requires the physical UNO Q and glasses. Software tests do not
establish DisplayPort compatibility, usable angular range, latency, or drift.

## Automated checks

```powershell
python -m unittest discover -s tests -p "test_*.py"
node tests/browser_logic.test.js
python tools/fake_hub.py --scenario motion --duration 1.5 --port 18080
```
