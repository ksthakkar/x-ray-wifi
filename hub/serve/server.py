"""HTTP and WebSocket delivery for wall-frame position estimates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any, Mapping

from aiohttp import WSMsgType, web

from .estimate import Estimate, EstimateValidationError


STATIC_DIR = Path(__file__).with_name("static")
DEFAULT_CONFIG = Path(__file__).parents[1] / "site" / "lab-partition-a.json"


class EstimateBroadcaster:
    """Validate estimates and fan the latest one out to display clients."""

    def __init__(self) -> None:
        self.clients: set[web.WebSocketResponse] = set()
        self.latest: Estimate | None = None
        self.last_publish_monotonic: float | None = None
        self.rejected_count = 0

    async def publish(self, payload: Estimate | Mapping[str, Any]) -> Estimate:
        try:
            estimate = payload if isinstance(payload, Estimate) else Estimate.from_mapping(payload)
        except EstimateValidationError:
            self.rejected_count += 1
            raise

        self.latest = estimate
        self.last_publish_monotonic = time.monotonic()
        message = json.dumps(estimate.to_dict(), separators=(",", ":"))
        dead: list[web.WebSocketResponse] = []
        for websocket in tuple(self.clients):
            try:
                await websocket.send_str(message)
            except (ConnectionError, RuntimeError):
                dead.append(websocket)
        for websocket in dead:
            self.clients.discard(websocket)
        return estimate

    async def add(self, websocket: web.WebSocketResponse) -> None:
        self.clients.add(websocket)
        if self.latest is not None:
            await websocket.send_str(
                json.dumps(self.latest.to_dict(), separators=(",", ":"))
            )

    def remove(self, websocket: web.WebSocketResponse) -> None:
        self.clients.discard(websocket)

    async def disconnect_all(self, message: str = "test disconnect") -> None:
        clients = tuple(self.clients)
        self.clients.clear()
        for websocket in clients:
            await websocket.close(code=1012, message=message.encode("utf-8"))

    def health(self) -> dict[str, Any]:
        age_ms = None
        if self.last_publish_monotonic is not None:
            age_ms = round((time.monotonic() - self.last_publish_monotonic) * 1000)
        return {
            "status": "ok",
            "websocket_clients": len(self.clients),
            "latest_t_us": self.latest.t_us if self.latest else None,
            "latest_age_ms": age_ms,
            "rejected_estimates": self.rejected_count,
        }


BROADCASTER_KEY: web.AppKey[EstimateBroadcaster] = web.AppKey(
    "estimate_broadcaster", EstimateBroadcaster
)
DISPLAY_CONFIG_KEY: web.AppKey[dict] = web.AppKey("display_config", dict)


def _load_display_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    if not isinstance(config, dict) or "display" not in config or "wall" not in config:
        raise ValueError(f"{path} must contain display and wall objects")
    return config


async def _index(_request: web.Request) -> web.FileResponse:
    return web.FileResponse(STATIC_DIR / "index.html")


async def _static_file(request: web.Request) -> web.FileResponse:
    filename = request.path.removeprefix("/")
    if filename not in {"app.js", "geometry.js"}:
        raise web.HTTPNotFound()
    return web.FileResponse(STATIC_DIR / filename)


async def _config(request: web.Request) -> web.Response:
    return web.json_response(request.app[DISPLAY_CONFIG_KEY])


async def _health(request: web.Request) -> web.Response:
    return web.json_response(request.app[BROADCASTER_KEY].health())


async def _publish(request: web.Request) -> web.Response:
    try:
        payload = await request.json()
        estimate = await request.app[BROADCASTER_KEY].publish(payload)
    except (json.JSONDecodeError, EstimateValidationError) as error:
        return web.json_response({"error": str(error)}, status=400)
    return web.json_response({"accepted_t_us": estimate.t_us}, status=202)


async def _websocket(request: web.Request) -> web.WebSocketResponse:
    websocket = web.WebSocketResponse(heartbeat=20)
    await websocket.prepare(request)
    broadcaster = request.app[BROADCASTER_KEY]
    await broadcaster.add(websocket)
    try:
        async for message in websocket:
            if message.type == WSMsgType.ERROR:
                break
    finally:
        broadcaster.remove(websocket)
    return websocket


async def _shutdown(app: web.Application) -> None:
    await app[BROADCASTER_KEY].disconnect_all("server shutdown")


def create_app(config_path: str | Path | None = None) -> web.Application:
    """Create the estimate display application without starting a server."""
    path = Path(config_path) if config_path else DEFAULT_CONFIG
    app = web.Application(client_max_size=64 * 1024)
    app[BROADCASTER_KEY] = EstimateBroadcaster()
    app[DISPLAY_CONFIG_KEY] = _load_display_config(path)
    app.router.add_get("/", _index)
    app.router.add_get("/app.js", _static_file, name="app")
    app.router.add_get("/geometry.js", _static_file, name="geometry")
    app.router.add_get("/api/config", _config)
    app.router.add_get("/healthz", _health)
    app.router.add_post("/api/estimates", _publish)
    app.router.add_get("/ws/estimates", _websocket)
    app.on_shutdown.append(_shutdown)
    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    web.run_app(create_app(args.config), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
