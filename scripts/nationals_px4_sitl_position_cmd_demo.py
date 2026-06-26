#!/usr/bin/env python3
import json
import math
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple

import rospy
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand

Point = Tuple[float, float, float]


@dataclass
class DemoWaypoint:
    name: str
    point: Point
    switch_radius: float
    best_distance: float = float("inf")
    reached: bool = False


@dataclass
class DemoResult:
    demo_only_position_cmd_driver: bool = True
    strict_super_planning: bool = False
    mavros_connected: bool = False
    px4ctrl_position_cmd_subscriber: bool = False
    offboard: bool = False
    armed: bool = False
    takeoff: bool = False
    position_cmd_continuous: bool = False
    attitude_continuous: bool = False
    reached_ring1: bool = False
    safe_exit: bool = False
    final_armed: bool = True
    final_mode: str = ""
    ring1_best_odom_distance: float = float("nan")
    waypoints: List[DemoWaypoint] = field(default_factory=list)


class PositionCmdDemo:
    def __init__(self) -> None:
        self.target_alt = float(rospy.get_param("~target_alt", 1.30))
        self.cmd_rate_hz = float(rospy.get_param("~cmd_rate_hz", 50.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 35.0))
        self.segment_timeout = float(rospy.get_param("~segment_timeout", 45.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 30.0))
        self.hover_hold_sec = float(rospy.get_param("~hover_hold_sec", 2.0))
        self.switch_radius = float(rospy.get_param("~switch_radius", 0.75))
        self.ring_switch_radius = float(rospy.get_param("~ring_switch_radius", 1.00))
        self.max_alt = float(rospy.get_param("~max_alt", 3.50))
        self.waypoints_path = str(rospy.get_param("~waypoints_path", ""))
        self.summary_path = str(rospy.get_param("~summary_path", ""))

        self.state: Optional[State] = None
        self.odom: Optional[Odometry] = None
        self.last_odom_time = rospy.Time(0)
        self.last_position_cmd_time = rospy.Time(0)
        self.last_attitude_time = rospy.Time(0)
        self.trajectory_id = 1
        self.result = DemoResult()

        self.cmd_pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=20)
        self.takeoff_land_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber("/position_cmd", PositionCommand, self._position_cmd_cb, queue_size=30)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self._attitude_cb, queue_size=30)

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        self.result.mavros_connected = bool(msg.connected)
        self.result.final_mode = msg.mode
        self.result.final_armed = bool(msg.armed)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = rospy.Time.now()

    def _position_cmd_cb(self, _msg: PositionCommand) -> None:
        self.last_position_cmd_time = rospy.Time.now()

    def _attitude_cb(self, _msg: AttitudeTarget) -> None:
        self.last_attitude_time = rospy.Time.now()

    def _connected(self) -> bool:
        return bool(self.state and self.state.connected)

    def _armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _offboard(self) -> bool:
        return bool(self.state and self.state.mode == "OFFBOARD")

    def _fresh(self, stamp: rospy.Time, max_age: float) -> bool:
        return (rospy.Time.now() - stamp).to_sec() <= max_age

    def _odom_fresh(self) -> bool:
        return self.odom is not None and self._fresh(self.last_odom_time, 1.0)

    def _attitude_fresh(self) -> bool:
        return self._fresh(self.last_attitude_time, 1.0)

    def _pos(self) -> Point:
        if self.odom is None:
            return (float("nan"), float("nan"), float("nan"))
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    @staticmethod
    def _dist(a: Point, b: Point) -> float:
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                rospy.loginfo("[position_cmd_demo] %s: OK", label)
                return True
            rate.sleep()
        rospy.logerr("[position_cmd_demo] %s: TIMEOUT", label)
        return False

    def _position_cmd(self, point: Point, yaw: float = 0.0) -> PositionCommand:
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.position.x = point[0]
        msg.position.y = point[1]
        msg.position.z = point[2]
        msg.yaw = yaw
        msg.yaw_dot = 0.0
        msg.trajectory_id = self.trajectory_id
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        msg.kx = [2.0, 2.0, 2.0]
        msg.kv = [1.5, 1.5, 1.5]
        return msg

    def _publish_cmd(self, point: Point) -> None:
        self.cmd_pub.publish(self._position_cmd(point))

    def _publish_for(self, point: Point, duration: float) -> None:
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(self.cmd_rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_cmd(point)
            rate.sleep()

    def _publish_takeoff_land(self, command: int, seconds: float = 1.0) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.takeoff_land_pub.publish(msg)
            rate.sleep()

    def _load_route(self) -> List[Point]:
        route: List[Point] = []
        if not self.waypoints_path:
            raise RuntimeError("~waypoints_path is required")
        with open(os.path.expanduser(self.waypoints_path), "r", encoding="utf-8") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3:
                    route.append((float(parts[0]), float(parts[1]), self.target_alt))
        if not route:
            raise RuntimeError(f"no route waypoints found in {self.waypoints_path}")
        return route

    def _build_waypoints(self, start: Point, route: List[Point]) -> List[DemoWaypoint]:
        ring1 = route[0]
        dx = ring1[0] - start[0]
        dy = ring1[1] - start[1]
        dist_xy = max(math.hypot(dx, dy), 1.0e-3)
        ux = dx / dist_xy
        uy = dy / dist_xy
        pre_backoff = min(0.80, dist_xy * 0.25)

        waypoints = [
            DemoWaypoint("hover", (start[0], start[1], self.target_alt), self.switch_radius),
            DemoWaypoint("mid1", (start[0] + dx * 0.35, start[1] + dy * 0.35, self.target_alt), self.switch_radius),
            DemoWaypoint("mid2", (start[0] + dx * 0.65, start[1] + dy * 0.65, self.target_alt), self.switch_radius),
            DemoWaypoint("pre_ring", (ring1[0] - ux * pre_backoff, ring1[1] - uy * pre_backoff, self.target_alt), self.switch_radius),
            DemoWaypoint("ring1", ring1, self.ring_switch_radius),
        ]

        if len(route) >= 2:
            next_target = route[1]
            ndx = next_target[0] - ring1[0]
            ndy = next_target[1] - ring1[1]
            ndist = max(math.hypot(ndx, ndy), 1.0e-3)
            waypoints.append(
                DemoWaypoint(
                    "post_ring_direction",
                    (ring1[0] + ndx / ndist * min(1.20, ndist), ring1[1] + ndy / ndist * min(1.20, ndist), self.target_alt),
                    self.switch_radius,
                )
            )
        return waypoints

    def _abort_reason(self, target: Point) -> Optional[str]:
        if not self._connected():
            return "mavros disconnected"
        if not self._odom_fresh():
            return "odom timeout"
        if not self._attitude_fresh():
            return "attitude setpoint timeout"
        pos = self._pos()
        if math.isfinite(pos[2]) and pos[2] > self.max_alt:
            return f"altitude too high: {pos[2]:.2f}m"
        return None

    def _run_waypoint(self, wp: DemoWaypoint) -> bool:
        rospy.loginfo(
            "[position_cmd_demo] goal %s -> x=%.3f y=%.3f z=%.3f switch_radius=%.2f",
            wp.name,
            wp.point[0],
            wp.point[1],
            wp.point[2],
            wp.switch_radius,
        )
        deadline = rospy.Time.now() + rospy.Duration(self.segment_timeout)
        rate = rospy.Rate(self.cmd_rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            reason = self._abort_reason(wp.point)
            if reason:
                rospy.logerr("[position_cmd_demo] abort at %s: %s", wp.name, reason)
                return False
            self._publish_cmd(wp.point)
            pos = self._pos()
            if all(math.isfinite(v) for v in pos):
                wp.best_distance = min(wp.best_distance, self._dist(pos, wp.point))
                if wp.best_distance <= wp.switch_radius:
                    wp.reached = True
                    rospy.loginfo("[position_cmd_demo] reached %s best_distance=%.3f", wp.name, wp.best_distance)
                    if wp.name == "hover":
                        self._publish_for(wp.point, self.hover_hold_sec)
                    return True
            rate.sleep()
        rospy.logerr("[position_cmd_demo] timeout at %s best_distance=%.3f", wp.name, wp.best_distance)
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

    def _wait_disarmed(self, timeout: float) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                self.result.final_armed = False
                return True
            rate.sleep()
        self.result.final_armed = self._armed()
        return not self.result.final_armed

    def safe_exit(self) -> None:
        rospy.loginfo("[position_cmd_demo] safe exit: LAND")
        rospy.sleep(1.0)
        self._publish_takeoff_land(TakeoffLand.LAND, seconds=1.0)
        deadline = rospy.Time.now() + rospy.Duration(self.land_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                self.result.safe_exit = True
                self.result.final_armed = False
                return
            if self._odom_fresh() and self._pos()[2] < 0.22:
                break
            rate.sleep()

        if not self._armed():
            self.result.safe_exit = True
            self.result.final_armed = False
            return
        if not (self._odom_fresh() and self._pos()[2] < 0.30):
            self.result.safe_exit = False
            self.result.final_armed = self._armed()
            rospy.logerr("[position_cmd_demo] refusing force disarm: vehicle is not confirmed near ground")
            return

        for _ in range(5):
            if (self._plain_disarm() or self._force_disarm()) and self._wait_disarmed(1.0):
                self.result.safe_exit = True
                return
            rospy.sleep(0.5)
        self.result.final_armed = self._armed()

    def _write_summary(self, result_code: int) -> None:
        self.result.position_cmd_continuous = self.result.position_cmd_continuous or self._fresh(self.last_position_cmd_time, 1.0)
        self.result.attitude_continuous = self.result.attitude_continuous or self._fresh(self.last_attitude_time, 1.0)
        ring = next((wp for wp in self.result.waypoints if wp.name == "ring1"), None)
        if ring is not None:
            self.result.ring1_best_odom_distance = ring.best_distance
            self.result.reached_ring1 = ring.reached
        self.result.final_armed = self._armed()
        self.result.final_mode = self.state.mode if self.state else ""

        rospy.loginfo(
            "[position_cmd_demo] SUMMARY demo_only_position_cmd_driver=%s strict_super_planning=%s "
            "mavros_connected=%s px4ctrl_position_cmd_subscriber=%s offboard=%s armed=%s takeoff=%s "
            "position_cmd_continuous=%s attitude_continuous=%s reached_ring1=%s safe_exit=%s "
            "final_armed=%s result_code=%d ring1_best_odom_distance=%.3f",
            self.result.demo_only_position_cmd_driver,
            self.result.strict_super_planning,
            self.result.mavros_connected,
            self.result.px4ctrl_position_cmd_subscriber,
            self.result.offboard,
            self.result.armed,
            self.result.takeoff,
            self.result.position_cmd_continuous,
            self.result.attitude_continuous,
            self.result.reached_ring1,
            self.result.safe_exit,
            self.result.final_armed,
            result_code,
            self.result.ring1_best_odom_distance,
        )
        for wp in self.result.waypoints:
            rospy.loginfo(
                "[position_cmd_demo] SUMMARY waypoint %s reached=%s best_distance=%.3f switch_radius=%.3f",
                wp.name,
                wp.reached,
                wp.best_distance,
                wp.switch_radius,
            )
        if self.summary_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.summary_path)), exist_ok=True)
            payload = asdict(self.result)
            payload["result_code"] = result_code
            payload["summary_path"] = os.path.abspath(self.summary_path)
            with open(os.path.abspath(self.summary_path), "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, sort_keys=True)
                f.write("\n")
            rospy.loginfo("[position_cmd_demo] summary_path=%s", os.path.abspath(self.summary_path))

    def run(self) -> int:
        rospy.logwarn("[position_cmd_demo] DEMO ONLY. Direct /position_cmd driver for PX4 SITL; not strict SUPER planning.")
        rospy.logwarn("[position_cmd_demo] SITL ONLY. Do not use on real hardware.")
        result_code = 6
        try:
            self.result.px4ctrl_position_cmd_subscriber = self.cmd_pub.get_num_connections() > 0
            if not self._wait_for(self._connected, 20.0, "/mavros/state connected=True"):
                result_code = 2
                return result_code
            if self._armed():
                rospy.logerr("[position_cmd_demo] refusing to start: already armed")
                result_code = 2
                return result_code
            if not self._wait_for(self._odom_fresh, 20.0, "fresh odom"):
                result_code = 2
                return result_code
            if not self._wait_for(lambda: self.cmd_pub.get_num_connections() > 0, 20.0, "px4ctrl subscribed to /position_cmd"):
                result_code = 2
                return result_code
            self.result.px4ctrl_position_cmd_subscriber = True
            if not self._wait_for(self._attitude_fresh, 20.0, "px4ctrl attitude setpoints"):
                result_code = 2
                return result_code

            start = self._pos()
            rospy.loginfo("[position_cmd_demo] initial odom x=%.3f y=%.3f z=%.3f", start[0], start[1], start[2])
            self._publish_takeoff_land(TakeoffLand.TAKEOFF, seconds=1.0)

            self.result.offboard = self._wait_for(self._offboard, 10.0, "OFFBOARD mode")
            self.result.armed = self._wait_for(self._armed, 10.0, "armed=True")
            if not (self.result.offboard and self.result.armed):
                result_code = 3
                return result_code

            route = self._load_route()
            self.result.waypoints = self._build_waypoints(start, route)
            self.result.takeoff = self._wait_for(
                lambda: abs(self._pos()[2] - self.target_alt) <= 0.25,
                self.takeoff_timeout,
                "takeoff hover",
            )
            if not self.result.takeoff:
                result_code = 4
                return result_code

            for wp in self.result.waypoints:
                if not self._run_waypoint(wp):
                    result_code = 5
                    return result_code

            self.result.position_cmd_continuous = self._fresh(self.last_position_cmd_time, 1.0)
            self.result.attitude_continuous = self._fresh(self.last_attitude_time, 1.0)
            ring = next((wp for wp in self.result.waypoints if wp.name == "ring1"), None)
            ok = (
                ring is not None
                and ring.reached
                and self.result.position_cmd_continuous
                and self.result.attitude_continuous
            )
            result_code = 0 if ok else 6
            return result_code
        finally:
            self.safe_exit()
            self._write_summary(result_code)


if __name__ == "__main__":
    rospy.init_node("nationals_px4_sitl_position_cmd_demo")
    sys.exit(PositionCmdDemo().run())
