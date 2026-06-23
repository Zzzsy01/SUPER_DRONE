#!/usr/bin/env python3
import json
import math
import os
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


Point = Tuple[float, float, float]
Bounds = Tuple[float, float, float, float]


def _numbers(value: Any) -> Optional[List[float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        out = []
        for item in value[:3]:
            if not isinstance(item, (int, float)):
                return None
            out.append(float(item))
        return out
    if isinstance(value, dict):
        keys = (("x", "y", "z"), ("X", "Y", "Z"))
        for triplet in keys:
            if all(k in value and isinstance(value[k], (int, float)) for k in triplet):
                return [float(value[k]) for k in triplet]
    return None


def _first_vec(obj: Dict[str, Any], keys: Sequence[str]) -> Optional[List[float]]:
    for key in keys:
        if key in obj:
            vec = _numbers(obj[key])
            if vec is not None:
                return vec
    return None


def _first_num(obj: Dict[str, Any], keys: Sequence[str], default: Optional[float] = None) -> Optional[float]:
    for key in keys:
        value = obj.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return default


def _add_box(points: List[Point], center: Sequence[float], size: Sequence[float], resolution: float) -> None:
    sx, sy, sz = [max(float(v), resolution) for v in size]
    cx, cy, cz = center
    nx = max(2, int(math.ceil(sx / resolution)) + 1)
    ny = max(2, int(math.ceil(sy / resolution)) + 1)
    nz = max(2, int(math.ceil(sz / resolution)) + 1)
    xs = [cx - sx * 0.5 + sx * i / (nx - 1) for i in range(nx)]
    ys = [cy - sy * 0.5 + sy * i / (ny - 1) for i in range(ny)]
    zs = [cz - sz * 0.5 + sz * i / (nz - 1) for i in range(nz)]
    for x in xs:
        for y in ys:
            points.append((x, y, cz - sz * 0.5))
            points.append((x, y, cz + sz * 0.5))
    for x in xs:
        for z in zs:
            points.append((x, cy - sy * 0.5, z))
            points.append((x, cy + sy * 0.5, z))
    for y in ys:
        for z in zs:
            points.append((cx - sx * 0.5, y, z))
            points.append((cx + sx * 0.5, y, z))


def _field_bounds(layout: Dict[str, Any]) -> Bounds:
    field = layout.get("field", {})
    size = field.get("size_m", {}) if isinstance(field, dict) else {}
    if isinstance(size, dict) and isinstance(size.get("x"), (int, float)) and isinstance(size.get("y"), (int, float)):
        return 0.0, float(size["x"]), 0.0, float(size["y"])
    if isinstance(size, (list, tuple)) and len(size) >= 2 and all(isinstance(v, (int, float)) for v in size[:2]):
        return 0.0, float(size[0]), 0.0, float(size[1])

    xs: List[float] = []
    ys: List[float] = []

    def collect(value: Any) -> None:
        if isinstance(value, dict):
            center = _first_vec(value, ("center_m", "center", "position_m", "position", "pose_m", "initial_pose_m", "xyz"))
            if center is not None:
                xs.append(center[0])
                ys.append(center[1])
            bounds = value.get("bounds_m")
            if isinstance(bounds, dict):
                for key in ("xmin", "xmax"):
                    if isinstance(bounds.get(key), (int, float)):
                        xs.append(float(bounds[key]))
                for key in ("ymin", "ymax"):
                    if isinstance(bounds.get(key), (int, float)):
                        ys.append(float(bounds[key]))
            for child in value.values():
                collect(child)
        elif isinstance(value, list):
            for child in value:
                collect(child)

    collect(layout)
    if xs and ys:
        return min(0.0, min(xs)), max(xs), min(0.0, min(ys)), max(ys)
    return 0.0, 8.0, 0.0, 12.0


def _add_boundary_walls(points: List[Point], bounds: Bounds, height: float, thickness: float,
                        resolution: float, inset: float) -> None:
    xmin, xmax, ymin, ymax = bounds
    xmin += inset
    xmax -= inset
    ymin += inset
    ymax -= inset
    cx = (xmin + xmax) * 0.5
    cy = (ymin + ymax) * 0.5
    sx = xmax - xmin
    sy = ymax - ymin
    zc = height * 0.5
    _add_box(points, (xmin - thickness * 0.5, cy, zc), (thickness, sy + 2.0 * thickness, height), resolution)
    _add_box(points, (xmax + thickness * 0.5, cy, zc), (thickness, sy + 2.0 * thickness, height), resolution)
    _add_box(points, (cx, ymin - thickness * 0.5, zc), (sx, thickness, height), resolution)
    _add_box(points, (cx, ymax + thickness * 0.5, zc), (sx, thickness, height), resolution)


def _add_cylinder(points: List[Point], center: Sequence[float], radius: float, height: float, resolution: float) -> None:
    radius = max(float(radius), resolution)
    height = max(float(height), resolution)
    cx, cy, cz = center
    n_theta = max(12, int(math.ceil(2.0 * math.pi * radius / resolution)))
    n_z = max(2, int(math.ceil(height / resolution)) + 1)
    for iz in range(n_z):
        z = cz - height * 0.5 + height * iz / (n_z - 1)
        for it in range(n_theta):
            theta = 2.0 * math.pi * it / n_theta
            points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta), z))


def _add_ring(points: List[Point], center: Sequence[float], radius: float, tube_radius: float, resolution: float) -> None:
    radius = max(float(radius), resolution)
    tube_radius = max(float(tube_radius), resolution * 0.5)
    cx, cy, cz = center
    n_theta = max(24, int(math.ceil(2.0 * math.pi * radius / resolution)))
    n_phi = max(6, int(math.ceil(2.0 * math.pi * tube_radius / resolution)))
    for it in range(n_theta):
        theta = 2.0 * math.pi * it / n_theta
        for ip in range(n_phi):
            phi = 2.0 * math.pi * ip / n_phi
            radial = radius + tube_radius * math.cos(phi)
            x = cx + tube_radius * math.sin(phi)
            y = cy + radial * math.cos(theta)
            z = cz + radial * math.sin(theta)
            points.append((x, y, z))


def _object_points(name: str, obj: Dict[str, Any], resolution: float) -> List[Point]:
    points: List[Point] = []
    lname = name.lower()
    center = _first_vec(obj, ("center_m", "center", "position_m", "position", "pose_m", "initial_pose_m", "xyz"))
    if center is None:
        min_corner = _first_vec(obj, ("min_m", "min", "bbox_min_m", "bbox_min"))
        max_corner = _first_vec(obj, ("max_m", "max", "bbox_max_m", "bbox_max"))
        if min_corner is not None and max_corner is not None:
            center = [(a + b) * 0.5 for a, b in zip(min_corner, max_corner)]
            size = [abs(b - a) for a, b in zip(min_corner, max_corner)]
            _add_box(points, center, size, resolution)
        return points

    size = _first_vec(obj, ("size_m", "dimensions_m", "dim_m", "scale_m", "size", "dimensions"))
    radius = _first_num(obj, ("radius_m", "radius", "trunk_radius_m", "r_m", "r"))
    height = _first_num(obj, ("height_m", "height", "h_m", "h"))

    if "double_arch_wall" in lname and all(k in obj for k in (
        "total_width_m", "total_height_m", "opening_width_m", "opening_height_m", "pillar_width_m", "depth_thickness_m"
    )):
        total_width = float(obj["total_width_m"])
        total_height = float(obj["total_height_m"])
        opening_height = float(obj["opening_height_m"])
        pillar_width = float(obj["pillar_width_m"])
        depth = float(obj["depth_thickness_m"])
        top_thickness = max(resolution, total_height - opening_height)
        pillar_z = opening_height * 0.5
        top_z = opening_height + top_thickness * 0.5
        half_total = total_width * 0.5
        side_x = -half_total + pillar_width * 0.5
        right_x = half_total - pillar_width * 0.5
        for dx in (side_x, right_x):
            _add_box(points, (center[0] + dx, center[1], pillar_z), (pillar_width, depth, opening_height), resolution)
        _add_box(points, (center[0], center[1], top_z), (total_width, depth, top_thickness), resolution)
    elif "ring" in lname:
        _add_ring(points, center, radius or 0.45, _first_num(obj, ("tube_radius_m", "tube_radius", "thickness_m"), 0.04), resolution)
    elif "tree" in lname:
        _add_cylinder(points, center, radius or 0.12, height or 1.6, resolution)
    elif radius is not None and height is not None:
        _add_cylinder(points, center, radius, height, resolution)
    elif size is not None:
        _add_box(points, center, size, resolution)
    elif radius is not None:
        _add_cylinder(points, center, radius, resolution, resolution)
    else:
        _add_box(points, center, (resolution, resolution, resolution), resolution)
    return points


def _walk_layout(value: Any, path: str, resolution: float, out: List[Point]) -> None:
    lower_path = path.lower()
    if "takeoff_zone" in lower_path or "landing_zone" in lower_path:
        return
    if isinstance(value, dict):
        out.extend(_object_points(path, value, resolution))
        for key, child in value.items():
            _walk_layout(child, f"{path}/{key}", resolution, out)
    elif isinstance(value, list):
        for i, child in enumerate(value):
            _walk_layout(child, f"{path}[{i}]", resolution, out)


def load_points(layout_path: str, resolution: float, include_boundary_walls: bool,
                boundary_height: float, boundary_resolution: float,
                boundary_thickness: float, boundary_inset: float) -> List[Point]:
    with open(os.path.expanduser(layout_path), "r", encoding="utf-8") as f:
        layout = json.load(f)
    points: List[Point] = []
    _walk_layout(layout, "layout", max(0.03, resolution), points)
    bounds = _field_bounds(layout)
    if include_boundary_walls:
        _add_boundary_walls(points, bounds, boundary_height, boundary_thickness,
                            max(0.03, boundary_resolution), boundary_inset)
    dedup = {}
    q = max(0.02, resolution * 0.5)
    for x, y, z in points:
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            dedup[(round(x / q), round(y / q), round(z / q))] = (x, y, z)
    return list(dedup.values())


def main() -> None:
    rospy.init_node("nationals_layout_cloud_publisher")
    layout_path = rospy.get_param("~layout_path", os.path.expanduser("~/ws/gezogo-guosai/layout.json"))
    frame_id = rospy.get_param("~frame_id", "world")
    publish_rate = float(rospy.get_param("~publish_rate", 5.0))
    point_resolution = float(rospy.get_param("~point_resolution", 0.10))
    include_boundary_walls = bool(rospy.get_param("~include_boundary_walls", True))
    boundary_height = float(rospy.get_param("~boundary_height", 3.2))
    boundary_resolution = float(rospy.get_param("~boundary_resolution", point_resolution))
    boundary_thickness = float(rospy.get_param("~boundary_thickness", 0.10))
    boundary_inset = float(rospy.get_param("~boundary_inset", 0.35))
    points = load_points(layout_path, point_resolution, include_boundary_walls,
                         boundary_height, boundary_resolution, boundary_thickness, boundary_inset)
    if not points:
        raise RuntimeError(f"no points generated from layout: {layout_path}")
    pub = rospy.Publisher("/cloud_registered", PointCloud2, queue_size=2)
    rate = rospy.Rate(publish_rate)
    rospy.loginfo("[nationals_layout_cloud_publisher] loaded %d points from %s", len(points), layout_path)
    while not rospy.is_shutdown():
        header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        pub.publish(pc2.create_cloud_xyz32(header, points))
        rate.sleep()


if __name__ == "__main__":
    main()
