#!/usr/bin/env python3
"""Generate README pick-and-lift GIF from episode-data-lab frames."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs' / 'assets'
DEFAULT_OUTPUT = ASSETS / 'm6-pick-and-lift.gif'


def candidate_episode_roots(explicit: Path | None) -> list[Path]:
    roots: list[Path] = []
    if explicit is not None:
        roots.append(explicit)
    if env_root := os.environ.get('EPISODE_DATA_LAB_ROOT'):
        roots.append(Path(env_root))
    roots.extend([
        Path.home() / 'robot-sim-lab' / 'robot-arm-episode-data-lab',
        ROOT.parent / 'robot-arm-episode-data-lab',
    ])
    unique: list[Path] = []
    for root in roots:
        resolved = root.expanduser().resolve()
        if resolved not in unique and resolved.exists():
            unique.append(resolved)
    return unique


def load_metadata(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def discover_success_episode(roots: list[Path], explicit_episode: Path | None) -> Path:
    candidates: list[Path] = []
    if explicit_episode is not None:
        candidates.append(explicit_episode.expanduser().resolve())
    for root in roots:
        candidates.extend(root.glob('dataset/v1/episode_*/metadata.json'))
        candidates.extend(root.glob('dataset/v1_backup_*/episode_*/metadata.json'))
        candidates.extend(root.glob('dataset_sample/episode_pick*/metadata.json'))

    scored: list[tuple[int, int, Path]] = []
    for metadata_path in candidates:
        episode_dir = metadata_path.parent
        images = sorted((episode_dir / 'images').glob('*.png'))
        if not images:
            continue
        metadata = load_metadata(metadata_path)
        if metadata.get('task_name') != 'pick_and_lift':
            continue
        if metadata.get('success') is not True:
            continue
        lift_mm = int(float(metadata.get('object_z_lift') or 0.0) * 1000)
        scored.append((lift_mm, len(images), episode_dir))

    if not scored:
        searched = ', '.join(str(root) for root in roots) or '<none>'
        raise FileNotFoundError(
            'No successful pick_and_lift episode with images/ found. '
            f'Searched: {searched}'
        )
    scored.sort(reverse=True)
    return scored[0][2]


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        '/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf',
    ]
    for candidate in candidates:
        path = Path(candidate)
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def phase_color(phase: str) -> tuple[int, int, int]:
    return {
        'reach': (96, 165, 250),
        'approach': (52, 211, 153),
        'close_gripper': (251, 191, 36),
        'lift': (248, 113, 113),
        'done': (74, 222, 128),
    }.get(phase, (203, 213, 225))


def annotate_frame(
    image: Image.Image,
    *,
    metadata: dict[str, Any],
    phase: str,
    step: int,
    total: int,
) -> Image.Image:
    canvas = image.convert('RGB').resize((840, 630), Image.Resampling.LANCZOS)
    draw = ImageDraw.Draw(canvas, 'RGBA')
    title_font = font(24, bold=True)
    body_font = font(18)
    small_font = font(15)

    draw.rectangle((0, 0, 840, 74), fill=(15, 23, 42, 225))
    draw.text((20, 12), 'M6 Pick-and-Lift Episode', fill=(255, 255, 255), font=title_font)
    instruction = metadata.get('language_instruction') or 'pick up the cube'
    draw.text((20, 43), str(instruction), fill=(203, 213, 225), font=small_font)

    success = 'SUCCESS' if metadata.get('success') else 'CHECK'
    lift_cm = float(metadata.get('object_z_lift') or 0.0) * 100.0
    badge = f'{success}  lift={lift_cm:.1f}cm'
    badge_w = int(draw.textlength(badge, font=body_font)) + 24
    draw.rounded_rectangle((840 - badge_w - 20, 17, 820, 55), radius=10, fill=(22, 163, 74, 230))
    draw.text((840 - badge_w - 8, 25), badge, fill=(255, 255, 255), font=body_font)

    bar_x0, bar_y0, bar_x1, bar_y1 = 20, 590, 820, 612
    draw.rounded_rectangle((bar_x0, bar_y0, bar_x1, bar_y1), radius=8, fill=(15, 23, 42, 190))
    progress = 0 if total <= 1 else step / float(total - 1)
    draw.rounded_rectangle(
        (bar_x0, bar_y0, bar_x0 + int((bar_x1 - bar_x0) * progress), bar_y1),
        radius=8,
        fill=(*phase_color(phase), 230),
    )
    draw.text((24, 563), f'phase: {phase}', fill=(255, 255, 255), font=body_font)
    draw.text((700, 563), f'{step + 1}/{total}', fill=(226, 232, 240), font=body_font)
    return canvas


def build_gif(
    episode_dir: Path,
    output: Path,
    *,
    frame_stride: int,
    duration_ms: int,
    max_frames: int,
) -> None:
    metadata = load_metadata(episode_dir / 'metadata.json')
    image_paths = sorted((episode_dir / 'images').glob('*.png'))
    phases = metadata.get('task_phases') or []
    if not image_paths:
        raise FileNotFoundError(f'No PNG frames found in {episode_dir / "images"}')

    selected = image_paths[::max(frame_stride, 1)]
    if len(selected) > max_frames:
        step = max(1, len(selected) // max_frames)
        selected = selected[::step][:max_frames]

    frames: list[Image.Image] = []
    total = len(image_paths)
    for image_path in selected:
        step_idx = int(image_path.stem)
        phase = phases[step_idx] if step_idx < len(phases) else 'episode'
        with Image.open(image_path) as raw:
            frames.append(annotate_frame(raw, metadata=metadata, phase=phase, step=step_idx, total=total))

    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
    )
    print(f'wrote {output} from {episode_dir} ({len(frames)} frames)')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode-data-lab-root', type=Path)
    parser.add_argument('--episode-dir', type=Path)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--frame-stride', type=int, default=2)
    parser.add_argument('--duration-ms', type=int, default=90)
    parser.add_argument('--max-frames', type=int, default=60)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    roots = candidate_episode_roots(args.episode_data_lab_root)
    episode_dir = discover_success_episode(roots, args.episode_dir)
    build_gif(
        episode_dir,
        args.output,
        frame_stride=args.frame_stride,
        duration_ms=args.duration_ms,
        max_frames=args.max_frames,
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
