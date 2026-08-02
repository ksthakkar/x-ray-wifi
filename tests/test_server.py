import json
import unittest

from aiohttp.test_utils import TestClient, TestServer

from hub.serve.estimate import EstimateValidationError
from hub.serve.server import EstimateBroadcaster, create_app
from tests.test_estimate import valid_payload


class FakeWebSocket:
    def __init__(self) -> None:
        self.messages: list[str] = []
        self.closed = False

    async def send_str(self, message: str) -> None:
        self.messages.append(message)

    async def close(self, **_kwargs) -> None:
        self.closed = True


class EstimateBroadcasterTests(unittest.IsolatedAsyncioTestCase):
    async def test_broadcasts_and_replays_latest_estimate(self) -> None:
        broadcaster = EstimateBroadcaster()
        first = FakeWebSocket()
        await broadcaster.add(first)
        await broadcaster.publish(valid_payload())
        self.assertEqual(json.loads(first.messages[0])["position_m"], [1.42, 1.85])

        late = FakeWebSocket()
        await broadcaster.add(late)
        self.assertEqual(len(late.messages), 1)
        self.assertEqual(json.loads(late.messages[0])["t_us"], valid_payload()["t_us"])

    async def test_rejected_estimate_is_counted_and_not_sent(self) -> None:
        broadcaster = EstimateBroadcaster()
        client = FakeWebSocket()
        await broadcaster.add(client)
        with self.assertRaises(EstimateValidationError):
            await broadcaster.publish({"present": "invalid"})
        self.assertEqual(client.messages, [])
        self.assertEqual(broadcaster.health()["rejected_estimates"], 1)

    async def test_disconnect_all_closes_clients(self) -> None:
        broadcaster = EstimateBroadcaster()
        client = FakeWebSocket()
        await broadcaster.add(client)
        await broadcaster.disconnect_all()
        self.assertTrue(client.closed)
        self.assertEqual(broadcaster.health()["websocket_clients"], 0)


class EstimateServerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.client = TestClient(TestServer(create_app()))
        await self.client.start_server()

    async def asyncTearDown(self) -> None:
        await self.client.close()

    async def test_serves_display_and_config(self) -> None:
        index = await self.client.get("/")
        self.assertEqual(index.status, 200)
        self.assertIn('id="overlay"', await index.text())
        config = await self.client.get("/api/config")
        self.assertEqual(config.status, 200)
        self.assertEqual((await config.json())["site"], "lab-partition-a")

    async def test_publish_rejects_malformed_estimate(self) -> None:
        response = await self.client.post("/api/estimates", json={"present": "invalid"})
        self.assertEqual(response.status, 400)
        health = await self.client.get("/healthz")
        self.assertEqual((await health.json())["rejected_estimates"], 1)

    async def test_websocket_receives_latest_estimate(self) -> None:
        response = await self.client.post("/api/estimates", json=valid_payload())
        self.assertEqual(response.status, 202)
        websocket = await self.client.ws_connect("/ws/estimates")
        message = await websocket.receive_json(timeout=1)
        self.assertEqual(message["position_m"], [1.42, 1.85])
        await websocket.close()


if __name__ == "__main__":
    unittest.main()
