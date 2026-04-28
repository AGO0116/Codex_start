#!/usr/bin/env python3
"""Create a simple 2D experiment map with a Unitree Go2 pose overlay (SVG)."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Pose:
    x: float
    y: float
    yaw_deg: float


@dataclass
class MapSpec:
    width_m: float
    height_m: float
    pose: Pose


DEFAULT_CONFIG = {
    "map": {
        "width_m": 10.0,
        "height_m": 8.0,
    },
    "go2_pose": {
        "x_m": 3.5,
        "y_m": 2.0,
        "yaw_deg": 45,
    },
}

OBSTACLES_M = [
    (2.0, 1.0, 3.0, 0.3),
    (6.0, 3.0, 0.4, 2.5),
    (1.0, 5.5, 2.8, 0.4),
]


def load_config(path: Path) -> MapSpec:
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(DEFAULT_CONFIG, indent=2), encoding="utf-8")

    cfg = json.loads(path.read_text(encoding="utf-8"))
    return MapSpec(
        width_m=float(cfg["map"]["width_m"]),
        height_m=float(cfg["map"]["height_m"]),
        pose=Pose(
            x=float(cfg["go2_pose"]["x_m"]),
            y=float(cfg["go2_pose"]["y_m"]),
            yaw_deg=float(cfg["go2_pose"]["yaw_deg"]),
        ),
    )


def to_px(x_m: float, y_m: float, scale: float, canvas_h: float) -> tuple[float, float]:
    x_px = x_m * scale
    y_px = canvas_h - (y_m * scale)
    return x_px, y_px


def render_svg(spec: MapSpec, output_path: Path) -> None:
    scale = 80.0
    canvas_w = spec.width_m * scale
    canvas_h = spec.height_m * scale

    px, py = to_px(spec.pose.x, spec.pose.y, scale, canvas_h)
    yaw_rad = math.radians(spec.pose.yaw_deg)
    arrow_len = 0.7 * scale
    tip_x = px + arrow_len * math.cos(yaw_rad)
    tip_y = py - arrow_len * math.sin(yaw_rad)

    obstacle_elements = []
    for ox, oy, ow, oh in OBSTACLES_M:
        ox_px, oy_px = to_px(ox, oy + oh, scale, canvas_h)
        obstacle_elements.append(
            f'<rect x="{ox_px:.1f}" y="{oy_px:.1f}" width="{ow*scale:.1f}" height="{oh*scale:.1f}" '
            'fill="#2f2f2f" opacity="0.7" />'
        )

    svg = f"""<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{canvas_w:.0f}\" height=\"{canvas_h:.0f}\" viewBox=\"0 0 {canvas_w:.0f} {canvas_h:.0f}\">
  <rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\" />
  <g stroke=\"#d1d5db\" stroke-width=\"1\"> 
    {''.join(f'<line x1="{i*scale:.1f}" y1="0" x2="{i*scale:.1f}" y2="{canvas_h:.1f}" />' for i in range(int(spec.width_m)+1))}
    {''.join(f'<line x1="0" y1="{j*scale:.1f}" x2="{canvas_w:.1f}" y2="{j*scale:.1f}" />' for j in range(int(spec.height_m)+1))}
  </g>
  <g>
    {''.join(obstacle_elements)}
  </g>
  <circle cx=\"{px:.1f}\" cy=\"{py:.1f}\" r=\"20\" fill=\"#56B4E9\" opacity=\"0.5\" />
  <circle cx=\"{px:.1f}\" cy=\"{py:.1f}\" r=\"10\" fill=\"#0072B2\" />
  <line x1=\"{px:.1f}\" y1=\"{py:.1f}\" x2=\"{tip_x:.1f}\" y2=\"{tip_y:.1f}\" stroke=\"#D55E00\" stroke-width=\"6\" stroke-linecap=\"round\" />
  <polygon points=\"{tip_x:.1f},{tip_y:.1f} {tip_x-12:.1f},{tip_y+8:.1f} {tip_x-12:.1f},{tip_y-8:.1f}\" fill=\"#D55E00\" />
  <text x=\"16\" y=\"28\" font-size=\"20\" font-family=\"Arial\" fill=\"#111827\">Unitree Go2 - Initial Map Placement</text>
</svg>
"""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(svg, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/go2_map.json"))
    parser.add_argument("--output", type=Path, default=Path("output/go2_map.svg"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    spec = load_config(args.config)
    render_svg(spec, args.output)
    print(f"Generated map image: {args.output}")


if __name__ == "__main__":
    main()
