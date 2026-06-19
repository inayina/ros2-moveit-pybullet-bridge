"""Serve built HOC frontend static files via aiohttp."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def resolve_frontend_dist(explicit: str = '') -> Path | None:
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if (path / 'index.html').is_file():
            return path
    candidates = [
        Path(__file__).resolve().parents[1] / 'frontend' / 'dist',
        Path.cwd() / 'frontend' / 'dist',
    ]
    for path in candidates:
        if (path / 'index.html').is_file():
            return path.resolve()
    return None


async def start_static_server(dist_dir: Path, port: int) -> Any:
    try:
        from aiohttp import web
    except ImportError as exc:
        raise RuntimeError('aiohttp required for static UI — pip install aiohttp') from exc

    dist_dir = dist_dir.resolve()

    async def index(_request: web.Request) -> web.Response:
        return web.FileResponse(dist_dir / 'index.html')

    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_static('/assets', dist_dir / 'assets', show_index=False)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', port)
    await site.start()
    logger.info('HOC static UI at http://0.0.0.0:%s (%s)', port, dist_dir)
    return runner
