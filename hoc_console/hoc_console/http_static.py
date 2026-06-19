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


def register_camera_routes(app, camera_getter) -> None:
    """Register JPEG snapshot and MJPEG stream routes on an aiohttp app."""
    from aiohttp import web

    async def latest_jpg(_request: web.Request) -> web.Response:
        data = camera_getter()
        if not data:
            raise web.HTTPServiceUnavailable(
                text='Waiting for /bridge/camera/image_compressed')
        return web.Response(
            body=data,
            content_type='image/jpeg',
            headers={'Cache-Control': 'no-store, no-cache, must-revalidate'},
        )

    async def mjpeg_stream(request: web.Request) -> web.StreamResponse:
        import asyncio

        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-cache, no-store, must-revalidate',
                'Connection': 'keep-alive',
                'Pragma': 'no-cache',
            },
        )
        await response.prepare(request)
        boundary = b'--frame\r\n'
        try:
            while True:
                if request.transport is not None and request.transport.is_closing():
                    break
                data = camera_getter()
                if data:
                    await response.write(
                        boundary
                        + b'Content-Type: image/jpeg\r\n\r\n'
                        + data
                        + b'\r\n'
                    )
                await asyncio.sleep(0.08)
        except (asyncio.CancelledError, ConnectionResetError, BrokenPipeError):
            pass
        return response

    app.router.add_get('/hoc/camera/latest.jpg', latest_jpg)
    app.router.add_get('/hoc/camera/mjpeg', mjpeg_stream)


async def start_static_server(
    dist_dir: Path,
    port: int,
    *,
    ws_handler=None,
    camera_getter=None,
) -> Any:
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
    if ws_handler is not None:
        app.router.add_get('/hoc-ws', ws_handler)
    if camera_getter is not None:
        register_camera_routes(app, camera_getter)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', port)
    await site.start()
    logger.info(
        'HOC static UI at http://0.0.0.0:%s (%s), WebSocket ws://0.0.0.0:%s/hoc-ws',
        port,
        dist_dir,
        port,
    )
    return runner
