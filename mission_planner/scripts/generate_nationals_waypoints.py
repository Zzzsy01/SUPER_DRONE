#!/usr/bin/env python3
import argparse
import json
import math
import os
from typing import Any, Dict, List, Optional, Sequence, Tuple


Bounds = Tuple[float, float, float, float]


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


def _find_double_arch(layout: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    found: Optional[Dict[str, Any]] = None

    def walk(value: Any, path: str) -> None:
        nonlocal found
        if found is not None:
            return
        if isinstance(value, dict):
            if "double_arch" in path.lower() and _center(value) is not None:
                found = value
                return
            for key, child in value.items():
                walk(child, f"{path}/{key}")
        elif isinstance(value, list):
            for i, child in enumerate(value):
                walk(child, f"{path}[{i}]")

    walk(layout, "layout")
    return found


def _arch_opening_target(point: List[float], arch: Optional[Dict[str, Any]], bounds: Bounds, margin: float) -> List[float]:
    if arch is None:
        return point
    center = _center(arch)
    if center is None:
        return point
    total_width = arch.get("total_width_m")
    opening_width = arch.get("opening_width_m")
    pillar_width = arch.get("pillar_width_m")
    depth = arch.get("depth_thickness_m")
    if not all(isinstance(v, (int, float)) for v in (total_width, opening_width, pillar_width, depth)):
        return point

    wall_ymin = center[1] - float(depth) * 0.5
    wall_ymax = center[1] + float(depth) * 0.5
    if not (point[1] <= wall_ymax):
        return point

    target = list(point)
    target[1] = min(point[1], wall_ymin - 0.35)
    return _clamp_to_bounds(target, bounds, margin)


def _field_bounds(layout: Dict[str, Any]) -> Bounds:
    field = layout.get("field", {})
    size = field.get("size_m", {}) if isinstance(field, dict) else {}
    if isinstance(size, dict) and isinstance(size.get("x"), (int, float)) and isinstance(size.get("y"), (int, float)):
        return 0.0, float(size["x"]), 0.0, float(size["y"])
    if isinstance(size, (list, tuple)) and len(size) >= 2 and all(isinstance(v, (int, float)) for v in size[:2]):
        return 0.0, float(size[0]), 0.0, float(size[1])
    return 0.0, 8.0, 0.0, 12.0


def _clamp_to_bounds(point: List[float], bounds: Bounds, margin: float) -> List[float]:
    xmin, xmax, ymin, ymax = bounds
    out = list(point)
    out[0] = min(max(out[0], xmin + margin), xmax - margin)
    out[1] = min(max(out[1], ymin + margin), ymax - margin)
    return out


def _in_bounds(point: List[float], bounds: Bounds, margin: float) -> bool:
    xmin, xmax, ymin, ymax = bounds
    return xmin + margin <= point[0] <= xmax - margin and ymin + margin <= point[1] <= ymax - margin


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate SUPER_DRONE nationals waypoint file from layout.json")
    parser.add_argument("--layout", default=os.path.expanduser("~/ws/gezogo-guosai/layout.json"))
    parser.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "..", "data", "nationals_seed_2026.txt"))
    parser.add_argument("--z", type=float, default=1.10)
    parser.add_argument("--switch-dis", type=float, default=0.40)
    parser.add_argument("--final-switch-dis", type=float, default=0.25)
    parser.add_argument("--landing-z", type=float, default=1.00)
    parser.add_argument("--field-margin", type=float, default=0.35)
    args = parser.parse_args()

    with open(os.path.expanduser(args.layout), "r", encoding="utf-8") as f:
        layout = json.load(f)
    bounds = _field_bounds(layout)

    rings: List[List[float]] = []
    _collect_named(layout, "layout", ("scoring_rings", "scoring_ring", "score_ring", "ring"), rings)
    rings = _sort_route(_unique(rings))
    if len(rings) < 4:
        raise RuntimeError(f"expected at least 4 scoring ring centers, found {len(rings)} in {args.layout}")
    arch = _find_double_arch(layout)
    rings = [_clamp_to_bounds(_arch_opening_target(p, arch, bounds, args.field_margin), bounds, args.field_margin)
             for p in rings[:4]]

    landing: List[List[float]] = []
    _collect_named(layout, "layout", ("landing", "land_zone", "landing_zone"), landing)
    landing = _unique(landing)
    if landing:
        final = landing[0]
    else:
        final = list(rings[-1])
        final[0] += 1.2
    final = _clamp_to_bounds(final, bounds, args.field_margin)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(os.path.abspath(args.output), "w", encoding="utf-8") as f:
        for i, p in enumerate(rings):
            f.write(f"{p[0]:.3f} {p[1]:.3f} {args.z:.3f} {args.switch_dis:.3f}\n")
            print(f"[generate_nationals_waypoints] waypoint {i}: ({p[0]:.3f}, {p[1]:.3f}, {args.z:.3f}) "
                  f"in_bounds={'OK' if _in_bounds(p, bounds, args.field_margin) else 'FAIL'}")
        f.write(f"{final[0]:.3f} {final[1]:.3f} {args.landing_z:.3f} {args.final_switch_dis:.3f}\n")
        print(f"[generate_nationals_waypoints] waypoint 4: ({final[0]:.3f}, {final[1]:.3f}, {args.landing_z:.3f}) "
              f"in_bounds={'OK' if _in_bounds(final, bounds, args.field_margin) else 'FAIL'}")
    print(f"[generate_nationals_waypoints] wrote {os.path.abspath(args.output)}")


if __name__ == "__main__":
    main()
