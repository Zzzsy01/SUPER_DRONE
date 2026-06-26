#!/usr/bin/env python3
import json
import math
import os
import sys
from typing import Any, List, Optional, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "mission_planner", "scripts"))
from nationals_frame_transform import distance_to_bounds, in_bounds_xy, parse_vec, transform_bounds, transform_point


Point = Tuple[float, float, float]


def _vec(value: Any) -> Optional[Point]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        if all(isinstance(v, (int, float)) for v in value[:3]):
            return (float(value[0]), float(value[1]), float(value[2]))
    if isinstance(value, dict) and all(k in value for k in ("x", "y", "z")):
        if all(isinstance(value[k], (int, float)) for k in ("x", "y", "z")):
            return (float(value["x"]), float(value["y"]), float(value["z"]))
    return None


def _center(obj: Any) -> Optional[Point]:
    if not isinstance(obj, dict):
        return None
    for key in ("center_m", "center", "position_m", "position", "pose_m", "initial_pose_m", "xyz"):
        if key in obj:
            got = _vec(obj[key])
            if got is not None:
                return got
    return None


def _collect_named(value: Any, path: str, needles: Sequence[str], out: List[Point]) -> None:
    if isinstance(value, dict):
        lname = path.lower()
        if any(needle in lname for needle in needles):
            c = _center(value)
            if c is not None:
                out.append(c)
        for key, child in value.items():
            _collect_named(child, f"{path}/{key}", needles, out)
    elif isinstance(value, list):
        for i, child in enumerate(value):
            _collect_named(child, f"{path}[{i}]", needles, out)


def _field_bounds(layout: Any) -> Tuple[float, float, float, float]:
    field = layout.get("field", {}) if isinstance(layout, dict) else {}
    size = field.get("size_m", {}) if isinstance(field, dict) else {}
    if isinstance(size, dict) and isinstance(size.get("x"), (int, float)) and isinstance(size.get("y"), (int, float)):
        return (0.0, float(size["x"]), 0.0, float(size["y"]))
    if isinstance(size, list) and len(size) >= 2:
        return (0.0, float(size[0]), 0.0, float(size[1]))
    return (0.0, 8.0, 0.0, 12.0)


class FrameCheck:
    def __init__(self) -> None:
        self.layout_path = str(rospy.get_param("~layout_path", ""))
        self.target_alt = float(rospy.get_param("~target_alt", 1.0))
        self.ring_alt = float(rospy.get_param("~ring_alt", 1.1))
        self.takeoff = bool(rospy.get_param("~takeoff", True))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 25.0))
        self.field_margin = float(rospy.get_param("~field_margin", 0.0))
        self.clear_radius = float(rospy.get_param("~clear_radius", 0.35))
        self.scale = parse_vec(str(rospy.get_param("~frame_scale", "-1,1,1")), (-1.0, 1.0, 1.0))
        self.offset = parse_vec(str(rospy.get_param("~frame_offset", "3,-1,0")), (3.0, -1.0, 0.0))
        self.state: Optional[State] = None
        self.odom: Optional[Odometry] = None
        self.cloud: Optional[PointCloud2] = None
        self.last_attitude = rospy.Time(0)
        self.takeoff_land_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self._attitude_cb, queue_size=20)
        rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_cb, queue_size=1)

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _attitude_cb(self, _msg: AttitudeTarget) -> None:
        self.last_attitude = rospy.Time.now()

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.cloud = msg

    def _pos(self) -> Point:
        if self.odom is None:
            return (float("nan"), float("nan"), float("nan"))
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                rospy.loginfo("[frame_check] %s: OK", label)
                return True
            rate.sleep()
        rospy.logerr("[frame_check] %s: TIMEOUT", label)
        return False

    def _publish_takeoff_land(self, command: int, seconds: float = 0.8) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.takeoff_land_pub.publish(msg)
            rate.sleep()

    def _armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _offboard(self) -> bool:
        return bool(self.state and self.state.mode == "OFFBOARD")

    def _land(self) -> None:
        self._publish_takeoff_land(TakeoffLand.LAND, 1.0)
        deadline = rospy.Time.now() + rospy.Duration(self.land_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                return
            if self._pos()[2] < 0.25:
                break
            rate.sleep()
        try:
            self.arm_srv(False)
        except Exception:
            try:
                self.command_srv(False, 400, 0, 0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass

    def _cloud_stats(self, hover: Point) -> Tuple[Point, Point, float, int]:
        if self.cloud is None:
            nan = (float("nan"), float("nan"), float("nan"))
            return nan, nan, float("nan"), 0
        min_p = [float("inf"), float("inf"), float("inf")]
        max_p = [float("-inf"), float("-inf"), float("-inf")]
        nearest = float("inf")
        near_count = 0
        for x, y, z in pc2.read_points(self.cloud, field_names=("x", "y", "z"), skip_nans=True):
            min_p[0], min_p[1], min_p[2] = min(min_p[0], x), min(min_p[1], y), min(min_p[2], z)
            max_p[0], max_p[1], max_p[2] = max(max_p[0], x), max(max_p[1], y), max(max_p[2], z)
            dxy = math.hypot(x - hover[0], y - hover[1])
            if abs(z - hover[2]) <= self.clear_radius:
                nearest = min(nearest, dxy)
                if dxy <= self.clear_radius:
                    near_count += 1
        return tuple(min_p), tuple(max_p), nearest, near_count

    def run(self) -> int:
        if not self._wait_for(lambda: bool(self.state and self.state.connected), 20.0, "/mavros/state connected"):
            return 2
        if not self._wait_for(lambda: self.odom is not None, 20.0, "odom"):
            return 2
        if not self._wait_for(lambda: self.cloud is not None, 20.0, "cloud_registered"):
            return 2
        if self.takeoff:
            if not self._wait_for(lambda: (rospy.Time.now() - self.last_attitude).to_sec() < 1.0, 20.0, "attitude setpoints"):
                return 2
            self._publish_takeoff_land(TakeoffLand.TAKEOFF, 1.0)
            if not self._wait_for(self._offboard, 10.0, "OFFBOARD"):
                return 3
            if not self._wait_for(self._armed, 10.0, "armed"):
                return 3
            if not self._wait_for(lambda: abs(self._pos()[2] - self.target_alt) <= 0.25, self.takeoff_timeout, "hover"):
                return 4
        try:
            with open(os.path.expanduser(self.layout_path), "r", encoding="utf-8") as f:
                layout = json.load(f)
            takeoffs: List[Point] = []
            rings: List[Point] = []
            _collect_named(layout, "layout", ("takeoff", "takeoff_zone", "start"), takeoffs)
            _collect_named(layout, "layout", ("scoring_rings", "scoring_ring", "score_ring", "ring"), rings)
            takeoff_raw = takeoffs[0] if takeoffs else (3.0, 1.0, 0.0)
            ring1_xy = sorted(set((round(p[0], 3), round(p[1], 3)) for p in rings))[0] if rings else (2.55, 6.25)
            ring1_raw = (ring1_xy[0], ring1_xy[1], self.ring_alt)
            takeoff_tf = transform_point(takeoff_raw, self.scale, self.offset)
            ring1_tf = transform_point(ring1_raw, self.scale, self.offset)
            raw_bounds = _field_bounds(layout)
            tf_bounds = transform_bounds(raw_bounds, self.scale, self.offset)
            hover = self._pos()
            cloud_min, cloud_max, nearest_cloud, near_count = self._cloud_stats(hover)
            boundary_distance = distance_to_bounds(hover, tf_bounds)
            hover_in_bounds = in_bounds_xy(hover, tf_bounds, self.field_margin)
            clear = near_count == 0

            rospy.loginfo("[frame_check] raw_takeoff_center=(%.3f, %.3f, %.3f)", *takeoff_raw)
            rospy.loginfo("[frame_check] transformed_takeoff_center=(%.3f, %.3f, %.3f)", *takeoff_tf)
            rospy.loginfo("[frame_check] px4_hover_odom=(%.3f, %.3f, %.3f)", *hover)
            rospy.loginfo("[frame_check] transformed_field_bounds=(%.3f, %.3f, %.3f, %.3f)", *tf_bounds)
            rospy.loginfo("[frame_check] hover_in_transformed_bounds=%s", hover_in_bounds)
            rospy.loginfo("[frame_check] distance_to_nearest_boundary=%.3f", boundary_distance)
            rospy.loginfo("[frame_check] raw_ring1=(%.3f, %.3f, %.3f)", *ring1_raw)
            rospy.loginfo("[frame_check] transformed_ring1=(%.3f, %.3f, %.3f)", *ring1_tf)
            rospy.loginfo("[frame_check] cloud_registered_min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)", *(cloud_min + cloud_max))
            rospy.loginfo("[frame_check] nearest_cloud_xy_at_hover_z=%.3f near_hover_points=%d clear=%s", nearest_cloud, near_count, clear)
            return 0 if hover_in_bounds and clear else 5
        finally:
            if self.takeoff:
                self._land()


if __name__ == "__main__":
    rospy.init_node("nationals_px4_sitl_frame_check")
    sys.exit(FrameCheck().run())
