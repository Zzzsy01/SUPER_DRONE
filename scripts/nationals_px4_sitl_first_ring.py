#!/usr/bin/env python3
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "mission_planner", "scripts"))
from nationals_frame_transform import parse_vec, transform_bounds

Point = Tuple[float, float, float]


@dataclass
class Waypoint:
    name: str
    point: Point
    switch_radius: float
    best_distance: float = float("inf")
    reached: bool = False


@dataclass
class Result:
    offboard: bool = False
    armed: bool = False
    takeoff: bool = False
    safe_exit: bool = False
    position_cmd_continuous: bool = False
    attitude_continuous: bool = False
    in_bounds: bool = True
    collision_free: bool = True
    fsm_backup_replan_failed: bool = False
    final_armed: bool = True
    waypoints: List[Waypoint] = field(default_factory=list)


class FirstRingMission:
    ERROR_PATTERNS = (
        "GeneratePolytopeFromLine failed",
        "generateBackupTrajectory return FAILED",
        "Cannot generate feasible backup sfc",
        "GenerateExpTrajectory failed",
        "Local start point is deeply occupied",
        "PathSearch for new path failed",
        "Odom below virtual ground",
    )

    def __init__(self) -> None:
        self.mission_mode = str(rospy.get_param("~mission_mode", "first_ring"))
        self.target_alt = float(rospy.get_param("~target_alt", 1.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.segment_timeout = float(rospy.get_param("~segment_timeout", 60.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 25.0))
        self.goal_publish_rate = float(rospy.get_param("~goal_publish_rate", 0.5))
        self.mid_switch_radius = float(rospy.get_param("~mid_switch_radius", 0.9))
        self.ring_switch_radius = float(rospy.get_param("~ring_switch_radius", 1.1))
        self.scoring_radius = float(rospy.get_param("~scoring_radius", 0.9))
        self.demo_spacing = float(rospy.get_param("~demo_spacing", 0.6))
        self.demo_switch_radius = float(rospy.get_param("~demo_switch_radius", 1.0))
        self.demo_min_reached = int(rospy.get_param("~demo_min_reached", 4))
        self.field_margin = float(rospy.get_param("~field_margin", 0.35))
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.25))
        self.waypoints_path = str(rospy.get_param("~waypoints_path", ""))
        self.layout_path = str(rospy.get_param("~layout_path", ""))
        self.transform_to_planning_frame = bool(rospy.get_param("~transform_to_planning_frame", True))
        self.frame_scale = parse_vec(str(rospy.get_param("~frame_scale", "-1,1,1")), (-1.0, 1.0, 1.0))
        self.frame_offset = parse_vec(str(rospy.get_param("~frame_offset", "3,-1,0")), (3.0, -1.0, 0.0))

        self.state: Optional[State] = None
        self.odom: Optional[Odometry] = None
        self.cloud: Optional[PointCloud2] = None
        self.last_odom_time = rospy.Time(0)
        self.last_position_cmd_time = rospy.Time(0)
        self.last_attitude_time = rospy.Time(0)
        self.fsm_errors: List[str] = []
        self.bounds = self._load_bounds(self.layout_path)
        self.result = Result()

        self.goal_pub = rospy.Publisher("/planning/click_goal", PoseStamped, queue_size=1, latch=True)
        self.takeoff_land_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber("/position_cmd", PositionCommand, self._position_cmd_cb, queue_size=30)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self._attitude_cb, queue_size=30)
        rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_cb, queue_size=1)
        rospy.Subscriber("/rosout_agg", Log, self._rosout_cb, queue_size=100)

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = rospy.Time.now()

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.cloud = msg

    def _position_cmd_cb(self, _msg: PositionCommand) -> None:
        self.last_position_cmd_time = rospy.Time.now()

    def _attitude_cb(self, _msg: AttitudeTarget) -> None:
        self.last_attitude_time = rospy.Time.now()

    def _rosout_cb(self, msg: Log) -> None:
        if any(pattern in msg.msg for pattern in self.ERROR_PATTERNS):
            self.fsm_errors.append(msg.msg)
            self.result.fsm_backup_replan_failed = True

    def _load_bounds(self, layout_path: str) -> Tuple[float, float, float, float]:
        if not layout_path or not os.path.isfile(os.path.expanduser(layout_path)):
            bounds = (0.0, 8.0, 0.0, 12.0)
            return transform_bounds(bounds, self.frame_scale, self.frame_offset) if self.transform_to_planning_frame else bounds
        with open(os.path.expanduser(layout_path), "r", encoding="utf-8") as f:
            layout = json.load(f)
        field = layout.get("field", {})
        size = field.get("size_m", {}) if isinstance(field, dict) else {}
        if isinstance(size, dict) and isinstance(size.get("x"), (int, float)) and isinstance(size.get("y"), (int, float)):
            bounds = (0.0, float(size["x"]), 0.0, float(size["y"]))
            return transform_bounds(bounds, self.frame_scale, self.frame_offset) if self.transform_to_planning_frame else bounds
        if isinstance(size, list) and len(size) >= 2:
            bounds = (0.0, float(size[0]), 0.0, float(size[1]))
            return transform_bounds(bounds, self.frame_scale, self.frame_offset) if self.transform_to_planning_frame else bounds
        bounds = (0.0, 8.0, 0.0, 12.0)
        return transform_bounds(bounds, self.frame_scale, self.frame_offset) if self.transform_to_planning_frame else bounds

    def _load_ring1(self) -> Point:
        return self._load_route()[0]

    def _load_route(self) -> List[Point]:
        route: List[Point] = []
        with open(self.waypoints_path, "r", encoding="utf-8") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3:
                    route.append((float(parts[0]), float(parts[1]), float(parts[2])))
        if not route:
            raise RuntimeError(f"no route waypoints found in {self.waypoints_path}")
        return route

    def _pos(self) -> Point:
        if self.odom is None:
            return (float("nan"), float("nan"), float("nan"))
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _dist(self, a: Point, b: Point) -> float:
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def _fresh(self, stamp: rospy.Time, max_age: float) -> bool:
        return (rospy.Time.now() - stamp).to_sec() <= max_age

    def _connected(self) -> bool:
        return bool(self.state and self.state.connected)

    def _armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _offboard(self) -> bool:
        return bool(self.state and self.state.mode == "OFFBOARD")

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                rospy.loginfo("[%s] %s: OK", self.mission_mode, label)
                return True
            rate.sleep()
        rospy.logerr("[%s] %s: TIMEOUT", self.mission_mode, label)
        return False

    def _publish_takeoff_land(self, command: int, seconds: float = 0.8) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.takeoff_land_pub.publish(msg)
            rate.sleep()

    def _publish_goal(self, point: Point) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = point[2]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _in_bounds(self, point: Point) -> bool:
        xmin, xmax, ymin, ymax = self.bounds
        return xmin + self.field_margin <= point[0] <= xmax - self.field_margin and ymin + self.field_margin <= point[1] <= ymax - self.field_margin

    def _collision_free(self, point: Point) -> bool:
        if self.cloud is None:
            return True
        checked = 0
        for x, y, z in pc2.read_points(self.cloud, field_names=("x", "y", "z"), skip_nans=True):
            checked += 1
            if checked % 5 != 0:
                continue
            if abs(z - point[2]) > self.robot_radius:
                continue
            if math.hypot(x - point[0], y - point[1]) <= self.robot_radius:
                return False
        return True

    def _build_waypoints(self, start: Point, ring: Point) -> List[Waypoint]:
        dx = ring[0] - start[0]
        dy = ring[1] - start[1]
        dist_xy = max(math.hypot(dx, dy), 1.0e-3)
        ux = dx / dist_xy
        uy = dy / dist_xy

        hover = (start[0], start[1], self.target_alt)
        mid1 = (start[0] + dx * 0.45, start[1] + dy * 0.45, min(ring[2], self.target_alt + 0.05))
        approach_backoff = min(1.0, dist_xy * 0.25)
        mid2 = (ring[0] - ux * approach_backoff, ring[1] - uy * approach_backoff, ring[2])
        return [
            Waypoint("hover_hold", hover, self.mid_switch_radius),
            Waypoint("mid1", mid1, self.mid_switch_radius),
            Waypoint("mid2", mid2, self.mid_switch_radius),
            Waypoint("ring1", ring, self.ring_switch_radius),
        ]

    def _build_demo_waypoints(self, start: Point, route: List[Point]) -> List[Waypoint]:
        waypoints: List[Waypoint] = [
            Waypoint("demo_hover_hold", (start[0], start[1], self.target_alt), self.demo_switch_radius)
        ]
        cursor = (start[0], start[1], self.target_alt)
        for target_i, raw_target in enumerate(route, start=1):
            target = (raw_target[0], raw_target[1], self.target_alt)
            dx = target[0] - cursor[0]
            dy = target[1] - cursor[1]
            dist_xy = math.hypot(dx, dy)
            steps = max(1, int(math.ceil(dist_xy / max(0.05, self.demo_spacing))))
            for step in range(1, steps + 1):
                alpha = step / steps
                point = (
                    cursor[0] + dx * alpha,
                    cursor[1] + dy * alpha,
                    self.target_alt,
                )
                waypoints.append(Waypoint(f"demo_{target_i:02d}_{step:02d}", point, self.demo_switch_radius))
            cursor = target
        return waypoints

    def _run_waypoint(self, wp: Waypoint) -> bool:
        rospy.loginfo(
            "[%s] goal %s -> x=%.3f y=%.3f z=%.3f switch_radius=%.2f",
            self.mission_mode,
            wp.name,
            wp.point[0],
            wp.point[1],
            wp.point[2],
            wp.switch_radius,
        )
        self.result.in_bounds = self.result.in_bounds and self._in_bounds(wp.point)
        self.result.collision_free = self.result.collision_free and self._collision_free(wp.point)
        deadline = rospy.Time.now() + rospy.Duration(self.segment_timeout)
        pub_period = rospy.Duration(1.0 / max(self.goal_publish_rate, 0.1))
        last_pub = rospy.Time(0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()
            if now - last_pub >= pub_period:
                self._publish_goal(wp.point)
                last_pub = now
            pos = self._pos()
            if all(math.isfinite(v) for v in pos):
                wp.best_distance = min(wp.best_distance, self._dist(pos, wp.point))
                if wp.best_distance <= wp.switch_radius:
                    wp.reached = True
                    rospy.loginfo("[%s] reached %s best_distance=%.3f", self.mission_mode, wp.name, wp.best_distance)
                    return True
            rate.sleep()
        rospy.logerr("[%s] timeout at %s best_distance=%.3f", self.mission_mode, wp.name, wp.best_distance)
        return False

    def _plain_disarm(self) -> bool:
        try:
            return bool(self.arm_srv(False).success)
        except rospy.ServiceException:
            return False

    def _force_disarm(self) -> bool:
        try:
            return bool(self.command_srv(False, 400, 0, 0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0).success)
        except rospy.ServiceException:
            return False

    def safe_exit(self) -> None:
        rospy.loginfo("[%s] safe exit: LAND", self.mission_mode)
        self._publish_takeoff_land(TakeoffLand.LAND, seconds=1.0)
        deadline = rospy.Time.now() + rospy.Duration(self.land_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                self.result.safe_exit = True
                self.result.final_armed = False
                return
            if self.odom is not None and self._pos()[2] < 0.20:
                break
            rate.sleep()
        if not self._armed():
            self.result.safe_exit = True
            self.result.final_armed = False
            return
        if self.odom is not None and self._pos()[2] < 0.25:
            for _ in range(5):
                if self._plain_disarm() or self._force_disarm():
                    rospy.sleep(0.5)
                    self.result.safe_exit = not self._armed()
                    self.result.final_armed = self._armed()
                    return
        self.result.final_armed = self._armed()

    def _print_summary(self) -> None:
        self.result.position_cmd_continuous = self._fresh(self.last_position_cmd_time, 1.0)
        self.result.attitude_continuous = self._fresh(self.last_attitude_time, 1.0)
        ring = next((wp for wp in self.result.waypoints if wp.name == "ring1"), None)
        ring_best = float("nan") if ring is None else ring.best_distance
        reached_count = sum(1 for wp in self.result.waypoints if wp.reached)
        rospy.loginfo(
            "[%s] SUMMARY offboard=%s armed=%s takeoff=%s safe_exit=%s final_armed=%s "
            "position_cmd_continuous=%s attitude_continuous=%s in_bounds=%s collision_free=%s "
            "fsm_backup_replan_failed=%s reached_waypoints=%d total_waypoints=%d "
            "ring1_best_odom_distance=%.3f scoring_radius=%.3f bounds=(%.3f,%.3f,%.3f,%.3f)",
            self.mission_mode,
            self.result.offboard,
            self.result.armed,
            self.result.takeoff,
            self.result.safe_exit,
            self.result.final_armed,
            self.result.position_cmd_continuous,
            self.result.attitude_continuous,
            self.result.in_bounds,
            self.result.collision_free,
            self.result.fsm_backup_replan_failed,
            reached_count,
            len(self.result.waypoints),
            ring_best,
            self.scoring_radius,
            self.bounds[0],
            self.bounds[1],
            self.bounds[2],
            self.bounds[3],
        )
        for wp in self.result.waypoints:
            rospy.loginfo("[%s] SUMMARY waypoint %s reached=%s best_distance=%.3f switch_radius=%.3f",
                          self.mission_mode, wp.name, wp.reached, wp.best_distance, wp.switch_radius)
        for msg in self.fsm_errors[-8:]:
            rospy.logwarn("[%s] SUMMARY fsm_error: %s", self.mission_mode, msg)

    def run(self) -> int:
        rospy.logwarn("[%s] SITL ONLY. Do not use on real hardware.", self.mission_mode)
        if self.mission_mode == "demo_full_route":
            rospy.logwarn("[demo_full_route] DEMO ONLY: Gazebo/PX4 SITL recording workflow; not a strict competition validation.")
        if not self._wait_for(self._connected, 20.0, "/mavros/state connected=True"):
            return 2
        if self._armed():
            rospy.logerr("[%s] refusing to start: already armed", self.mission_mode)
            return 2
        if not self._wait_for(lambda: self.odom is not None and self._fresh(self.last_odom_time, 1.0), 20.0, "fresh odom"):
            return 2
        if not self._wait_for(lambda: self._fresh(self.last_attitude_time, 1.0), 20.0, "px4ctrl attitude setpoints"):
            return 2

        start_xy = self._pos()
        rospy.loginfo("[%s] initial odom x=%.3f y=%.3f z=%.3f", self.mission_mode, start_xy[0], start_xy[1], start_xy[2])
        self._publish_takeoff_land(TakeoffLand.TAKEOFF, seconds=1.0)
        result_code = 6
        try:
            self.result.offboard = self._wait_for(self._offboard, 10.0, "OFFBOARD mode")
            self.result.armed = self._wait_for(self._armed, 10.0, "armed=True")
            if not (self.result.offboard and self.result.armed):
                result_code = 3
                return result_code
            self.result.takeoff = self._wait_for(lambda: abs(self._pos()[2] - self.target_alt) <= 0.25, self.takeoff_timeout, "takeoff hover")
            if not self.result.takeoff:
                result_code = 4
                return result_code

            hover_start = self._pos()
            if self.mission_mode == "demo_full_route":
                self.result.waypoints = self._build_demo_waypoints(hover_start, self._load_route())
            else:
                ring1 = self._load_ring1()
                self.result.waypoints = self._build_waypoints(hover_start, ring1)
            for wp in self.result.waypoints:
                if not self._run_waypoint(wp):
                    result_code = 5
                    return result_code

            self.result.position_cmd_continuous = self._fresh(self.last_position_cmd_time, 1.0)
            self.result.attitude_continuous = self._fresh(self.last_attitude_time, 1.0)
            reached_count = sum(1 for wp in self.result.waypoints if wp.reached)
            if self.mission_mode == "demo_full_route":
                ok = (
                    all(wp.reached for wp in self.result.waypoints)
                    and reached_count >= min(len(self.result.waypoints), self.demo_min_reached)
                    and self.result.position_cmd_continuous
                    and self.result.attitude_continuous
                    and self.result.in_bounds
                )
            else:
                ok = (
                    all(wp.reached for wp in self.result.waypoints)
                    and self.result.position_cmd_continuous
                    and self.result.attitude_continuous
                    and self.result.in_bounds
                    and self.result.collision_free
                    and not self.result.fsm_backup_replan_failed
                )
            result_code = 0 if ok else 6
            return result_code
        finally:
            self.safe_exit()
            self._print_summary()


if __name__ == "__main__":
    rospy.init_node("nationals_px4_sitl_first_ring")
    sys.exit(FirstRingMission().run())
