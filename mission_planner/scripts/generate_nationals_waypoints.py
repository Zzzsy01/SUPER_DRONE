#!/usr/bin/env python3
import argparse
import json
import math
import os
from typing import Any, Dict, List, Optional, Sequence


def _vec(value: Any) -> Optional[List[float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        if all(isinstance(v, (int, float)) for v in value[:3]):
            return [float(v) for v in value[:3]]
    if isinstance(value, dict) and all(k in value for k in ("x", "y", "z")):
        if all(isinstance(value[k], (int, float)) for k in ("x", "y", "z")):
            return [float(value[k]) for k in ("x", "y", "z")]
    return None


def _center(obj: Dict[str, Any]) -> Optional[List[float]]:
    for key in ("center_m", "center", "position_m", "position", "pose_m", "xyz"):
        if key in obj:
            got = _vec(obj[key])
            if got is not None:
                return got
    return None


def _collect_named(value: Any, path: str, needles: Sequence[str], out: List[List[float]]) -> None:
    if isinstance(value, dict):
        lname = path.lower()
        if any(needle in lname for needle in needles):
            center = _center(value)
            if center is not None:
                out.append(center)
        for key, child in value.items():
            _collect_named(child, f"{path}/{key}", needles, out)
    elif isinstance(value, list):
        for i, child in enumerate(value):
            _collect_named(child, f"{path}[{i}]", needles, out)


def _unique(points: List[List[float]]) -> List[List[float]]:
    seen = set()
    out = []
    for p in points:
        key = tuple(round(v, 3) for v in p)
        if key not in seen:
            seen.add(key)
            out.append(p)
    return out


def _sort_route(points: List[List[float]]) -> List[List[float]]:
    return sorted(points, key=lambda p: (p[0], p[1], p[2]))


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate SUPER_DRONE nationals waypoint file from layout.json")
    parser.add_argument("--layout", default=os.path.expanduser("~/ws/gezogo-guosai/layout.json"))
    parser.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "..", "data", "nationals_seed_2026.txt"))
    parser.add_argument("--z", type=float, default=1.10)
    parser.add_argument("--switch-dis", type=float, default=0.40)
    parser.add_argument("--final-switch-dis", type=float, default=0.25)
    parser.add_argument("--landing-z", type=float, default=1.00)
    args = parser.parse_args()

    with open(os.path.expanduser(args.layout), "r", encoding="utf-8") as f:
        layout = json.load(f)

    rings: List[List[float]] = []
    _collect_named(layout, "layout", ("scoring_rings", "scoring_ring", "score_ring", "ring"), rings)
    rings = _sort_route(_unique(rings))
    if len(rings) < 4:
        raise RuntimeError(f"expected at least 4 scoring ring centers, found {len(rings)} in {args.layout}")
    rings = rings[:4]

    landing: List[List[float]] = []
    _collect_named(layout, "layout", ("landing", "land_zone", "landing_zone"), landing)
    landing = _unique(landing)
    if landing:
        final = landing[0]
    else:
        final = list(rings[-1])
        final[0] += 1.2

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(os.path.abspath(args.output), "w", encoding="utf-8") as f:
        for p in rings:
            f.write(f"{p[0]:.3f} {p[1]:.3f} {args.z:.3f} {args.switch_dis:.3f}\n")
        f.write(f"{final[0]:.3f} {final[1]:.3f} {args.landing_z:.3f} {args.final_switch_dis:.3f}\n")
    print(f"[generate_nationals_waypoints] wrote {os.path.abspath(args.output)}")


if __name__ == "__main__":
    main()
