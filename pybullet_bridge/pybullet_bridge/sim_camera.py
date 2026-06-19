"""Offscreen PyBullet camera capture for HOC live preview."""

from __future__ import annotations

import io
from typing import Any


def capture_jpeg(
    pybullet_module: Any,
    client_id: int,
    *,
    width: int = 640,
    height: int = 480,
    quality: int = 82,
) -> bytes | None:
    """Render the current simulation and return JPEG bytes."""
    p = pybullet_module
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.35, 0.0, 0.55],
        distance=1.85,
        yaw=48,
        pitch=-22,
        roll=0,
        upAxisIndex=2,
    )
    proj = p.computeProjectionMatrixFOV(
        fov=58,
        aspect=float(width) / float(height),
        nearVal=0.08,
        farVal=4.5,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width,
        height,
        view,
        proj,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=client_id,
    )
    if rgba is None:
        return None

    try:
        from PIL import Image
    except ImportError:
        return None

    rgb = bytes(rgba)
    image = Image.frombytes('RGBA', (width, height), rgb).convert('RGB')
    buf = io.BytesIO()
    image.save(buf, format='JPEG', quality=quality, optimize=True)
    return buf.getvalue()
