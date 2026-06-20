"""Preflight checks for HOC launch files."""

from __future__ import annotations

import socket
from typing import Iterable


def port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.3)
        return sock.connect_ex((host, port)) == 0


def require_free_ports(ports: Iterable[int], *, label: str) -> None:
    busy = [port for port in ports if port_open('127.0.0.1', port)]
    if not busy:
        return
    ports_txt = ', '.join(str(p) for p in busy)
    raise RuntimeError(
        f'{label}: port(s) {ports_txt} already in use. '
        'Stop stale HOC/Vite processes, then relaunch:\n'
        '  pkill -f "/hoc_console/hoc_server"\n'
        '  pkill -f "vite --host"\n'
        '  ss -tlnp | grep -E ":5173|:8765"')
