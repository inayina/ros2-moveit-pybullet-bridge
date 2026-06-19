"""WebSocket broadcast hub for HOC clients."""

from __future__ import annotations

import asyncio
import json
from typing import Any


class WsHub:
    """Manage WebSocket clients and broadcast JSON frames."""

    def __init__(self) -> None:
        self._clients: set = set()

    def register(self, websocket) -> None:
        self._clients.add(websocket)

    def unregister(self, websocket) -> None:
        self._clients.discard(websocket)

    async def broadcast(self, message_type: str, payload: dict[str, Any]) -> None:
        if not self._clients:
            return
        frame = json.dumps({
            'type': message_type,
            'payload': payload,
        })
        dead: list = []
        for ws in self._clients:
            try:
                await ws.send(frame)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.unregister(ws)
