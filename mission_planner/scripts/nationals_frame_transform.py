#!/usr/bin/env python3
import math
from typing import Iterable, Sequence, Tuple


Point = Tuple[float, float, float]
Bounds = Tuple[float, float, float, float]


def parse_vec(value: str, default: Point) -> Point:
    if not value:
        return default
    parts = [p.strip() for p in value.replace(";", ",").split(",") if p.strip()]
    if len(parts) != 3:
        return default
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return default


def transform_point(point: Sequence[float], scale: Point = (-1.0, 1.0, 1.0),
                    offset: Point = (3.0, -1.0, 0.0)) -> Point:
    return (
        float(point[0]) * scale[0] + offset[0],
        float(point[1]) * scale[1] + offset[1],
        float(point[2]) * scale[2] + offset[2],
    )


def transform_points(points: Iterable[Sequence[float]], scale: Point = (-1.0, 1.0, 1.0),
                     offset: Point = (3.0, -1.0, 0.0)):
    for point in points:
        yield transform_point(point, scale, offset)


def transform_bounds(bounds: Bounds, scale: Point = (-1.0, 1.0, 1.0),
                     offset: Point = (3.0, -1.0, 0.0)) -> Bounds:
    xmin, xmax, ymin, ymax = bounds
    corners = [
        transform_point((xmin, ymin, 0.0), scale, offset),
        transform_point((xmin, ymax, 0.0), scale, offset),
        transform_point((xmax, ymin, 0.0), scale, offset),
        transform_point((xmax, ymax, 0.0), scale, offset),
    ]
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return min(xs), max(xs), min(ys), max(ys)


def in_bounds_xy(point: Sequence[float], bounds: Bounds, margin: float = 0.0) -> bool:
    xmin, xmax, ymin, ymax = bounds
    return xmin + margin <= point[0] <= xmax - margin and ymin + margin <= point[1] <= ymax - margin


def distance_to_bounds(point: Sequence[float], bounds: Bounds) -> float:
    xmin, xmax, ymin, ymax = bounds
    x, y = float(point[0]), float(point[1])
    if xmin <= x <= xmax and ymin <= y <= ymax:
        return min(x - xmin, xmax - x, y - ymin, ymax - y)
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return -math.hypot(dx, dy)
