"""WebSocket broadcast hub for HOC clients."""

from __future__ import annotations

import json
import time
from typing import Any


TOPIC_RISK = '/risk/status'
TOPIC_METRICS = '/monitor/distribution_metrics'
TOPIC_TRACKING = '/monitor/tracking_error'

TOPIC_TYPE_MAP = {
    TOPIC_RISK: 'risk_status',
    TOPIC_METRICS: 'distribution_metrics',
    TOPIC_TRACKING: 'tracking_error',
}

TYPE_TOPIC_MAP = {v: k for k, v in TOPIC_TYPE_MAP.items()}


class WsHub:
    """Manage WebSocket clients and broadcast JSON frames."""

    def __init__(self) -> None:
        self._clients: dict[Any, set[str]] = {}

    def register(self, websocket) -> None:
        self._clients[websocket] = {TOPIC_RISK, TOPIC_METRICS, TOPIC_TRACKING}

    def unregister(self, websocket) -> None:
        self._clients.pop(websocket, None)

    def subscribe(self, websocket, topics: list[str]) -> None:
        if websocket in self._clients:
            self._clients[websocket] = set(topics)

    @staticmethod
    def _timestamp() -> dict[str, int]:
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        return {'sec': sec, 'nanosec': nanosec}

    async def broadcast(self, message_type: str, payload: dict[str, Any]) -> None:
        if not self._clients:
            return
        topic = TYPE_TOPIC_MAP.get(message_type, message_type)
        await self._send_to_subscribers(message_type, topic, payload)

    async def broadcast_topic(self, topic: str, payload: dict[str, Any]) -> None:
        if not self._clients:
            return
        message_type = TOPIC_TYPE_MAP.get(topic, 'data')
        await self._send_to_subscribers(message_type, topic, payload)

    async def send_to(self, websocket, frame: dict[str, Any]) -> None:
        await websocket.send(json.dumps(frame))

    async def _send_to_subscribers(
        self,
        message_type: str,
        topic: str,
        payload: dict[str, Any],
    ) -> None:
        ts = self._timestamp()
        data_frame = json.dumps({
            'type': 'data',
            'topic': topic,
            'timestamp': ts,
            'payload': payload,
        })
        dead: list = []
        for ws, topics in self._clients.items():
            if topic not in topics and message_type not in topics:
                continue
            try:
                await ws.send(data_frame)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.unregister(ws)
