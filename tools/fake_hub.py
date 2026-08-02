"""Run the AR display with deterministic synthetic localization estimates.

Usage from the repository root:
    python tools/fake_hub.py --scenario all
"""

from __future__ import annotations

import argparse
import asyncio
import math
from pathlib import Path
import sys
import time

from aiohttp import web

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hub.serve.estimate import EstimateValidationError  # noqa: E402
from hub.serve.server import BROADCASTER_KEY, create_app  # noqa: E402


def synthetic_estimate(elapsed: float, scenario: str) -> dict:
    phase = elapsed * 0.55
    x_m = 1.8 + 1.25 * math.sin(phase)
    y_m = 1.7 + 0.7 * math.cos(phase * 0.72)
    present = 0.94
    spread = 0.08 + 0.05 * (1.0 + math.sin(phase * 0.4))

    cycle_t = elapsed % 36.0
    if scenario in {"all", "absence"} and 16.0 <= cycle_t < 20.0:
        present = 0.08
    if scenario in {"all", "uncertainty"} and 20.0 <= cycle_t < 28.0:
        spread = 0.08 + (cycle_t - 20.0) * 0.09

    return {
        "t_us": time.time_ns() // 1000,
        "site": "lab-partition-a",
        "present": present,
        "position_m": [x_m, y_m],
        "covariance": [[spread, spread * 0.18], [spread * 0.18, spread * 1.8]],
        "height_m": None,
        "nodes_online": [1, 2, 4, 5],
        "nodes_expected": [1, 2, 3, 4, 5],
        "quality": max(0.2, min(0.96, 1.0 - spread * 0.45)),
        "model": "fake-hub@1",
    }


async def generate(app: web.Application, scenario: str, rate_hz: float) -> None:
    broadcaster = app[BROADCASTER_KEY]
    started = time.monotonic()
    cycle_number = -1
    disconnected_cycle = -1
    malformed_cycle = -1

    while True:
        elapsed = time.monotonic() - started
        cycle = int(elapsed // 36.0)
        cycle_t = elapsed % 36.0
        if cycle != cycle_number:
            cycle_number = cycle
            print(f"Synthetic scenario cycle {cycle + 1}")

        should_pause = scenario in {"all", "stale"} and 28.0 <= cycle_t < 30.0
        if not should_pause:
            await broadcaster.publish(synthetic_estimate(elapsed, scenario))

        if scenario == "all" and cycle_t >= 30.0 and disconnected_cycle != cycle:
            disconnected_cycle = cycle
            print("Closing clients once to exercise automatic reconnect")
            await broadcaster.disconnect_all("fake reconnect test")

        if scenario == "all" and cycle_t >= 31.0 and malformed_cycle != cycle:
            malformed_cycle = cycle
            try:
                await broadcaster.publish({"present": "invalid"})
            except EstimateValidationError as error:
                print(f"Rejected malformed test estimate as expected: {error}")

        await asyncio.sleep(1.0 / rate_hz)


async def run(args: argparse.Namespace) -> None:
    app = create_app(args.config)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.port)
    await site.start()
    display_host = "localhost" if args.host == "0.0.0.0" else args.host
    print(f"Fixed-view display: http://{display_host}:{args.port}")
    print(f"Scenario: {args.scenario}; estimate rate: {args.rate_hz:g} Hz")
    generator = asyncio.create_task(generate(app, args.scenario, args.rate_hz))
    try:
        if args.duration > 0:
            await asyncio.sleep(args.duration)
        else:
            await asyncio.Event().wait()
    finally:
        generator.cancel()
        await asyncio.gather(generator, return_exceptions=True)
        await runner.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument(
        "--scenario",
        choices=("motion", "absence", "uncertainty", "stale", "all"),
        default="all",
    )
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "hub" / "site" / "lab-partition-a.json",
    )
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
