"""Unit tests for WebSocket hub."""

from __future__ import annotations

import asyncio
import json

from hoc_console.ws_hub import TOPIC_METRICS, TOPIC_RISK, WsHub


class _FakeWebSocket:
    def __init__(self) -> None:
        self.sent: list[str] = []

    async def send(self, data: str) -> None:
        self.sent.append(data)


def test_broadcast_data_frame_to_subscribers():
    hub = WsHub()
    ws = _FakeWebSocket()
    hub.register(ws)

    asyncio.run(hub.broadcast('risk_status', {'level': 1, 'composite_score': 0.2}))

    assert len(ws.sent) == 1
    frame = json.loads(ws.sent[0])
    assert frame['type'] == 'data'
    assert frame['topic'] == TOPIC_RISK
    assert frame['payload']['level'] == 1


def test_subscribe_filters_topics():
    hub = WsHub()
    ws = _FakeWebSocket()
    hub.register(ws)
    hub.subscribe(ws, [TOPIC_METRICS])

    asyncio.run(hub.broadcast('risk_status', {'level': 2}))
    asyncio.run(hub.broadcast('distribution_metrics', {'kl_divergence_mean': 0.1}))

    assert len(ws.sent) == 1
    frame = json.loads(ws.sent[0])
    assert frame['topic'] == TOPIC_METRICS


def test_send_to_single_client():
    hub = WsHub()
    ws = _FakeWebSocket()
    asyncio.run(hub.send_to(ws, {'type': 'pong'}))
    assert json.loads(ws.sent[0])['type'] == 'pong'


def test_camera_frame_reaches_all_subscribers():
    hub = WsHub()
    ws = _FakeWebSocket()
    hub.register(ws)
    hub.subscribe(ws, [TOPIC_METRICS])

    asyncio.run(hub.broadcast('camera_frame', {'image_b64': 'abc123'}))

    assert len(ws.sent) == 1
    frame = json.loads(ws.sent[0])
    assert frame['type'] == 'camera_frame'
    assert frame['payload']['image_b64'] == 'abc123'
