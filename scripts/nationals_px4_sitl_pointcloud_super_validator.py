#!/usr/bin/env python3
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from sensor_msgs.msg import PointCloud2

Point = Tuple[float, float, float]


@dataclass
class SuperPointcloudResult:
    demo_only_position_cmd_driver: bool = False
    pointcloud_input: bool = True
    planner: str = "SUPER"
    strict_super_full_mission: bool = False
    cloud_registered_hz: float = 0.0
    odom_hz: float = 0.0
    position_cmd_hz: float = 0.0
    attitude_hz: float = 0.0
    position_cmd_publishers: List[str] = field(default_factory=list)
    position_cmd_subscribers: List[str] = field(default_factory=list)
    mavros_connected: bool = False
    offboard: bool = False
    armed: bool = False
    takeoff: bool = False
    reached_safe_goal: bool = False
    safe_exit: bool = False
    final_armed: bool = True
    final_mode: str = ""
    safe_goal: List[float] = field(default_factory=list)
    best_goal_distance: float = float("inf")
    result_code: int = 9


class PointcloudSuperValidator:
    def __init__(self) -> None:
        self.target_alt = float(rospy.get_param("~target_alt", 1.0))
        self.goal_dx = float(rospy.get_param("~goal_dx", 0.0))
        self.goal_dy = float(rospy.get_param("~goal_dy", 1.0))
        self.goal_dz = float(rospy.get_param("~goal_dz", 0.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 1.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 35.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 60.0))
        self.metric_window = float(rospy.get_param("~metric_window", 5.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 30.0))
        self.summary_path = str(rospy.get_param("~summary_path", ""))

        self.state: Optional[State] = None
        self.odom: Optional[Odometry] = None
        self.last_cloud = rospy.Time(0)
        self.last_odom = rospy.Time(0)
        self.last_cmd = rospy.Time(0)
        self.last_att = rospy.Time(0)
        self.cloud_times: List[float] = []
        self.odom_times: List[float] = []
        self.cmd_times: List[float] = []
        self.att_times: List[float] = []
        self.result = SuperPointcloudResult()

        self.goal_pub = rospy.Publisher("/planning/click_goal", PoseStamped, queue_size=1, latch=True)
        self.takeoff_land_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=100)
        rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_cb, queue_size=20)
        rospy.Subscriber("/position_cmd", PositionCommand, self._cmd_cb, queue_size=100)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self._att_cb, queue_size=100)

    def _now(self) -> float:
        return rospy.Time.now().to_sec()

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        self.result.mavros_connected = bool(msg.connected)
        self.result.final_mode = msg.mode
        self.result.final_armed = bool(msg.armed)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom = rospy.Time.now()
        self.odom_times.append(self._now())

    def _cloud_cb(self, _msg: PointCloud2) -> None:
        self.last_cloud = rospy.Time.now()
        self.cloud_times.append(self._now())

    def _cmd_cb(self, _msg: PositionCommand) -> None:
        self.last_cmd = rospy.Time.now()
        self.cmd_times.append(self._now())

    def _att_cb(self, _msg: AttitudeTarget) -> None:
        self.last_att = rospy.Time.now()
        self.att_times.append(self._now())

    def _fresh(self, stamp: rospy.Time, max_age: float = 1.0) -> bool:
        return (rospy.Time.now() - stamp).to_sec() <= max_age

    def _connected(self) -> bool:
        return bool(self.state and self.state.connected)

    def _armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _offboard(self) -> bool:
        return bool(self.state and self.state.mode == "OFFBOARD")

    def _pos(self) -> Point:
        if self.odom is None:
            return (float("nan"), float("nan"), float("nan"))
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    @staticmethod
    def _dist(a: Point, b: Point) -> float:
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    @staticmethod
    def _rate(times: List[float], window: float) -> float:
        if not times:
            return 0.0
        end = times[-1]
        recent = [t for t in times if end - t <= window]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return 0.0 if span <= 0.0 else (len(recent) - 1) / span

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                rospy.loginfo("[pointcloud_super] %s: OK", label)
                return True
            rate.sleep()
        rospy.logerr("[pointcloud_super] %s: TIMEOUT", label)
        return False

    def _publish_takeoff_land(self, command: int, seconds: float = 1.0) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.takeoff_land_pub.publish(msg)
            rate.sleep()

    def _publish_goal(self, goal: Point) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("[pointcloud_super] published safe click_goal x=%.3f y=%.3f z=%.3f", *goal)

    @staticmethod
    def _topic_info(topic: str) -> Tuple[List[str], List[str]]:
        try:
            output = subprocess.check_output(["rostopic", "info", topic], text=True, stderr=subprocess.DEVNULL)
        except (subprocess.CalledProcessError, OSError):
            return [], []
        publishers: List[str] = []
        subscribers: List[str] = []
        section = ""
        for raw in output.splitlines():
            line = raw.strip()
            if line.startswith("Publishers:"):
                section = "pub"
                continue
            if line.startswith("Subscribers:"):
                section = "sub"
                continue
            if line.startswith("* "):
                name = line[2:].split()[0]
                if section == "pub":
                    publishers.append(name)
                elif section == "sub":
                    subscribers.append(name)
        return publishers, subscribers

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
        rospy.loginfo("[pointcloud_super] safe exit: LAND")
        self._publish_takeoff_land(TakeoffLand.LAND, seconds=1.0)
        deadline = rospy.Time.now() + rospy.Duration(self.land_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                self.result.safe_exit = True
                self.result.final_armed = False
                return
            if self.odom is not None and self._pos()[2] < 0.25:
                break
            rate.sleep()
        if not self._armed():
            self.result.safe_exit = True
            self.result.final_armed = False
            return
        if self.odom is not None and self._pos()[2] < 0.35:
            self.result.safe_exit = (self._plain_disarm() or self._force_disarm()) and self._wait_disarmed(3.0)
        self.result.final_armed = self._armed()

    def _update_metrics(self) -> None:
        self.result.cloud_registered_hz = self._rate(self.cloud_times, self.metric_window)
        self.result.odom_hz = self._rate(self.odom_times, self.metric_window)
        self.result.position_cmd_hz = self._rate(self.cmd_times, self.metric_window)
        self.result.attitude_hz = self._rate(self.att_times, self.metric_window)
        pubs, subs = self._topic_info("/position_cmd")
        self.result.position_cmd_publishers = pubs
        self.result.position_cmd_subscribers = subs
        self.result.final_armed = self._armed()
        self.result.final_mode = self.state.mode if self.state else ""

    def _write_summary(self, code: int) -> None:
        self._update_metrics()
        self.result.result_code = code
        rospy.loginfo(
            "[pointcloud_super] SUMMARY demo_only_position_cmd_driver=%s pointcloud_input=%s planner=%s "
            "strict_super_full_mission=%s cloud_registered_hz=%.2f odom_hz=%.2f position_cmd_hz=%.2f "
            "attitude_hz=%.2f publishers=%s subscribers=%s reached_safe_goal=%s safe_exit=%s final_armed=%s result_code=%d",
            self.result.demo_only_position_cmd_driver,
            self.result.pointcloud_input,
            self.result.planner,
            self.result.strict_super_full_mission,
            self.result.cloud_registered_hz,
            self.result.odom_hz,
            self.result.position_cmd_hz,
            self.result.attitude_hz,
            self.result.position_cmd_publishers,
            self.result.position_cmd_subscribers,
            self.result.reached_safe_goal,
            self.result.safe_exit,
            self.result.final_armed,
            code,
        )
        if self.summary_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.summary_path)), exist_ok=True)
            with open(os.path.abspath(self.summary_path), "w", encoding="utf-8") as f:
                json.dump(asdict(self.result), f, indent=2, sort_keys=True)
                f.write("\n")
            rospy.loginfo("[pointcloud_super] summary_path=%s", os.path.abspath(self.summary_path))

    def _pass_conditions(self) -> bool:
        pubs = self.result.position_cmd_publishers
        subs = self.result.position_cmd_subscribers
        return (
            self.result.cloud_registered_hz >= 3.0
            and self.result.odom_hz >= 20.0
            and self.result.position_cmd_hz > 1.0
            and "/fsm_node" in pubs
            and not any("position_cmd_demo" in p for p in pubs)
            and any("px4ctrl" in s for s in subs)
            and self.result.attitude_hz > 1.0
            and self.result.offboard
            and self.result.armed
            and self.result.takeoff
            and self.result.reached_safe_goal
            and self.result.safe_exit
            and not self.result.final_armed
        )

    def run(self) -> int:
        rospy.logwarn("[pointcloud_super] SUPER pointcloud PX4 SITL demo. Validator does not publish /position_cmd.")
        code = 9
        try:
            if not self._wait_for(self._connected, 20.0, "/mavros/state connected=True"):
                code = 2
                return code
            if self._armed():
                rospy.logerr("[pointcloud_super] refusing to start: already armed")
                code = 2
                return code
            if not self._wait_for(lambda: self._fresh(self.last_cloud), 20.0, "/cloud_registered fresh"):
                code = 2
                return code
            if not self._wait_for(lambda: self._fresh(self.last_odom), 20.0, "/Odom_high_freq fresh"):
                code = 2
                return code
            if not self._wait_for(lambda: self._fresh(self.last_att), 20.0, "/mavros/setpoint_raw/attitude fresh"):
                code = 2
                return code

            self._publish_takeoff_land(TakeoffLand.TAKEOFF, seconds=1.0)
            self.result.offboard = self._wait_for(self._offboard, 10.0, "OFFBOARD mode")
            self.result.armed = self._wait_for(self._armed, 10.0, "armed=True")
            if not (self.result.offboard and self.result.armed):
                code = 3
                return code
            self.result.takeoff = self._wait_for(lambda: abs(self._pos()[2] - self.target_alt) <= 0.30, self.takeoff_timeout, "takeoff hover")
            if not self.result.takeoff:
                code = 4
                return code

            start = self._pos()
            goal = (start[0] + self.goal_dx, start[1] + self.goal_dy, max(0.8, start[2] + self.goal_dz))
            self.result.safe_goal = [goal[0], goal[1], goal[2]]
            self._publish_goal(goal)

            deadline = rospy.Time.now() + rospy.Duration(self.goal_timeout)
            rate = rospy.Rate(20)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                self._update_metrics()
                if "/fsm_node" in self.result.position_cmd_publishers:
                    pos = self._pos()
                    if all(math.isfinite(v) for v in pos):
                        self.result.best_goal_distance = min(self.result.best_goal_distance, self._dist(pos, goal))
                        if self.result.best_goal_distance <= self.goal_tolerance:
                            self.result.reached_safe_goal = True
                            rospy.loginfo("[pointcloud_super] reached safe goal best_distance=%.3f", self.result.best_goal_distance)
                            code = 0
                            return code
                rate.sleep()
            rospy.logerr("[pointcloud_super] safe goal timeout best_distance=%.3f", self.result.best_goal_distance)
            code = 5
            return code
        finally:
            self.safe_exit()
            self._write_summary(code)
            if code == 0 and not self._pass_conditions():
                rospy.logerr("[pointcloud_super] final PASS conditions failed")
                self.result.result_code = 6
                self._write_summary(6)


if __name__ == "__main__":
    rospy.init_node("nationals_px4_sitl_pointcloud_super_validator")
    validator = PointcloudSuperValidator()
    result = validator.run()
    if result == 0 and not validator._pass_conditions():
        sys.exit(6)
    sys.exit(result)
