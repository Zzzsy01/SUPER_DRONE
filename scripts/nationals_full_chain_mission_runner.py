#!/usr/bin/env python3
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

Point = Tuple[float, float, float]


@dataclass
class MissionWaypoint:
    name: str
    point: Point
    switch_radius: float
    best_distance: float = float("inf")
    reached: bool = False


@dataclass
class FullChainSummary:
    result_code: int = 9
    cloud_source: str = "layout_generated"
    planner: str = "SUPER"
    physics_model: str = "iris"
    mavlink_connection: str = "udp://:14540@127.0.0.1:14557"
    waypoints: Dict[str, List[float]] = field(default_factory=dict)
    reached_ring1: bool = False
    reached_ring2: bool = False
    reached_ring3: bool = False
    reached_ring4: bool = False
    reached_final: bool = False
    landed: bool = False
    safe_exit: bool = False
    final_armed: bool = True
    position_cmd_publishers: List[str] = field(default_factory=list)
    position_cmd_subscribers: List[str] = field(default_factory=list)
    cloud_registered_hz: float = 0.0
    odom_hz: float = 0.0
    position_cmd_hz: float = 0.0
    attitude_hz: float = 0.0
    rviz_started: bool = False
    rviz_config: str = ""
    bag_recorded: bool = False
    bag_path: Optional[str] = None
    completed_stage: str = "smoke"
    offboard: bool = False
    armed: bool = False
    takeoff: bool = False


class FullChainMissionRunner:
    def __init__(self) -> None:
        self.target_alt = float(rospy.get_param("~target_alt", 1.0))
        self.waypoints_path = str(rospy.get_param("~waypoints_path", ""))
        self.summary_path = str(rospy.get_param("~summary_path", ""))
        self.rviz_started = bool(rospy.get_param("~rviz_started", False))
        self.rviz_config = str(rospy.get_param("~rviz_config", ""))
        self.bag_recorded = bool(rospy.get_param("~bag_recorded", False))
        bag_path = str(rospy.get_param("~bag_path", ""))
        self.bag_path = bag_path if bag_path else None
        self.mission_max_rings = str(rospy.get_param("~mission_max_rings", "1")).lower()
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 35.0))
        self.segment_timeout = float(rospy.get_param("~segment_timeout", 75.0))
        self.subgoal_timeout = float(rospy.get_param("~subgoal_timeout", 25.0))
        self.subgoal_step = float(rospy.get_param("~subgoal_step", 1.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 35.0))
        self.goal_publish_rate = float(rospy.get_param("~goal_publish_rate", 0.5))
        self.metric_window = float(rospy.get_param("~metric_window", 5.0))

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
        self.position_cmd_path = Path()
        self.position_cmd_path.header.frame_id = "world"
        self.executed_path = Path()
        self.executed_path.header.frame_id = "world"
        self.summary = FullChainSummary(
            rviz_started=self.rviz_started,
            rviz_config=self.rviz_config,
            bag_recorded=self.bag_recorded,
            bag_path=self.bag_path,
        )
        self.mission_waypoints = self._load_waypoints()
        self.summary.waypoints = {wp.name: [wp.point[0], wp.point[1], wp.point[2]] for wp in self.mission_waypoints}

        self.goal_pub = rospy.Publisher("/planning/click_goal", PoseStamped, queue_size=1, latch=True)
        self.takeoff_land_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.marker_pub = rospy.Publisher("/nationals_mission/waypoints", MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/nationals_mission/status", MarkerArray, queue_size=1, latch=True)
        self.cmd_path_pub = rospy.Publisher("/nationals_mission/position_cmd_path", Path, queue_size=1)
        self.exec_path_pub = rospy.Publisher("/nationals_mission/executed_path", Path, queue_size=1)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=100)
        rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_cb, queue_size=20)
        rospy.Subscriber("/position_cmd", PositionCommand, self._cmd_cb, queue_size=100)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self._att_cb, queue_size=100)

    def _load_waypoints(self) -> List[MissionWaypoint]:
        names = ["ring1", "ring2", "ring3", "ring4", "final"]
        out: List[MissionWaypoint] = []
        with open(os.path.expanduser(self.waypoints_path), "r", encoding="utf-8") as f:
            for i, line in enumerate(f):
                parts = line.split()
                if len(parts) < 3 or i >= len(names):
                    continue
                radius = float(parts[3]) if len(parts) >= 4 else (1.0 if i < 4 else 0.6)
                out.append(MissionWaypoint(names[i], (float(parts[0]), float(parts[1]), float(parts[2])), radius))
        if len(out) < 5:
            raise RuntimeError(f"expected 5 nationals waypoints, got {len(out)} from {self.waypoints_path}")
        return out

    def _now(self) -> float:
        return rospy.Time.now().to_sec()

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        self.summary.final_armed = bool(msg.armed)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom = rospy.Time.now()
        self.odom_times.append(self._now())
        p = msg.pose.pose.position
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        pose.pose.position.z = p.z
        pose.pose.orientation = msg.pose.pose.orientation
        self.executed_path.header.stamp = pose.header.stamp
        self.executed_path.poses.append(pose)
        self.executed_path.poses = self.executed_path.poses[-500:]

    def _cloud_cb(self, _msg: PointCloud2) -> None:
        self.last_cloud = rospy.Time.now()
        self.cloud_times.append(self._now())

    def _cmd_cb(self, msg: PositionCommand) -> None:
        self.last_cmd = rospy.Time.now()
        self.cmd_times.append(self._now())
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z
        pose.pose.orientation.w = 1.0
        self.position_cmd_path.header.stamp = pose.header.stamp
        self.position_cmd_path.poses.append(pose)
        self.position_cmd_path.poses = self.position_cmd_path.poses[-500:]

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

    def _topic_info(self, topic: str) -> Tuple[List[str], List[str]]:
        try:
            output = subprocess.check_output(["rostopic", "info", topic], text=True, stderr=subprocess.DEVNULL)
        except (subprocess.CalledProcessError, OSError):
            return [], []
        pubs: List[str] = []
        subs: List[str] = []
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
                node = line[2:].split()[0]
                if section == "pub":
                    pubs.append(node)
                elif section == "sub":
                    subs.append(node)
        return pubs, subs

    def _update_metrics(self) -> None:
        self.summary.cloud_registered_hz = self._rate(self.cloud_times, self.metric_window)
        self.summary.odom_hz = self._rate(self.odom_times, self.metric_window)
        self.summary.position_cmd_hz = self._rate(self.cmd_times, self.metric_window)
        self.summary.attitude_hz = self._rate(self.att_times, self.metric_window)
        pubs, subs = self._topic_info("/position_cmd")
        self.summary.position_cmd_publishers = pubs
        self.summary.position_cmd_subscribers = subs
        self.summary.final_armed = self._armed()

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_visuals()
            if predicate():
                rospy.loginfo("[full_chain] %s: OK", label)
                return True
            rate.sleep()
        rospy.logerr("[full_chain] %s: TIMEOUT", label)
        return False

    def _publish_takeoff_land(self, command: int, seconds: float = 1.0) -> None:
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

    def _selected_waypoints(self) -> List[MissionWaypoint]:
        if self.mission_max_rings == "full":
            return self.mission_waypoints
        try:
            count = max(1, min(4, int(self.mission_max_rings)))
        except ValueError:
            count = 1
        return self.mission_waypoints[:count]

    def _publish_visuals(self, active: Optional[str] = None) -> None:
        arr = MarkerArray()
        now = rospy.Time.now()
        for i, wp in enumerate(self.mission_waypoints):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "world"
            m.ns = "nationals_waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wp.point
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.28 if wp.name != active else 0.42
            if wp.reached:
                m.color.g = 1.0
            elif wp.name == active:
                m.color.r = 1.0
                m.color.g = 0.8
            else:
                m.color.b = 1.0
                m.color.g = 0.4
            m.color.a = 0.95
            arr.markers.append(m)

            t = Marker()
            t.header.stamp = now
            t.header.frame_id = "world"
            t.ns = "nationals_waypoint_labels"
            t.id = 100 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp.point[0]
            t.pose.position.y = wp.point[1]
            t.pose.position.z = wp.point[2] + 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.28
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = wp.name
            arr.markers.append(t)
        self.marker_pub.publish(arr)

        status = Marker()
        status.header.stamp = now
        status.header.frame_id = "world"
        status.ns = "nationals_status"
        status.id = 1
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        pos = self._pos()
        status.pose.position.x = pos[0] if math.isfinite(pos[0]) else 0.0
        status.pose.position.y = pos[1] if math.isfinite(pos[1]) else 0.0
        status.pose.position.z = (pos[2] if math.isfinite(pos[2]) else 1.0) + 0.8
        status.pose.orientation.w = 1.0
        status.scale.z = 0.3
        status.color.r = 1.0
        status.color.g = 1.0
        status.color.a = 1.0
        status.text = f"stage={self.summary.completed_stage} active={active or 'none'}"
        self.status_pub.publish(MarkerArray(markers=[status]))
        self.cmd_path_pub.publish(self.position_cmd_path)
        self.exec_path_pub.publish(self.executed_path)

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
        rospy.loginfo("[full_chain] safe exit: LAND")
        self._publish_takeoff_land(TakeoffLand.LAND, seconds=1.0)
        deadline = rospy.Time.now() + rospy.Duration(self.land_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_visuals()
            if not self._armed():
                self.summary.safe_exit = True
                self.summary.final_armed = False
                self.summary.landed = True
                return
            if self.odom is not None and self._pos()[2] < 0.25:
                break
            rate.sleep()
        if not self._armed():
            self.summary.safe_exit = True
            self.summary.final_armed = False
            self.summary.landed = True
            return
        if self.odom is not None and self._pos()[2] < 0.35:
            self.summary.safe_exit = (self._plain_disarm() or self._force_disarm())
            rospy.sleep(0.5)
            self.summary.final_armed = self._armed()
            self.summary.landed = not self.summary.final_armed
        else:
            if self._force_disarm():
                rospy.sleep(0.5)
            self.summary.final_armed = self._armed()
            self.summary.safe_exit = not self.summary.final_armed
            self.summary.landed = not self.summary.final_armed

    def _write_summary(self, code: int) -> None:
        self._update_metrics()
        self.summary.result_code = code
        reached = {wp.name: wp.reached for wp in self.mission_waypoints}
        self.summary.reached_ring1 = reached.get("ring1", False)
        self.summary.reached_ring2 = reached.get("ring2", False)
        self.summary.reached_ring3 = reached.get("ring3", False)
        self.summary.reached_ring4 = reached.get("ring4", False)
        self.summary.reached_final = reached.get("final", False)
        rospy.loginfo(
            "[full_chain] SUMMARY result_code=%d completed_stage=%s cloud_hz=%.2f odom_hz=%.2f "
            "position_cmd_hz=%.2f attitude_hz=%.2f publishers=%s subscribers=%s "
            "reached=(%s,%s,%s,%s,%s) safe_exit=%s final_armed=%s rviz_started=%s bag_recorded=%s",
            code,
            self.summary.completed_stage,
            self.summary.cloud_registered_hz,
            self.summary.odom_hz,
            self.summary.position_cmd_hz,
            self.summary.attitude_hz,
            self.summary.position_cmd_publishers,
            self.summary.position_cmd_subscribers,
            self.summary.reached_ring1,
            self.summary.reached_ring2,
            self.summary.reached_ring3,
            self.summary.reached_ring4,
            self.summary.reached_final,
            self.summary.safe_exit,
            self.summary.final_armed,
            self.summary.rviz_started,
            self.summary.bag_recorded,
        )
        if self.summary_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.summary_path)), exist_ok=True)
            with open(os.path.abspath(self.summary_path), "w", encoding="utf-8") as f:
                json.dump(asdict(self.summary), f, indent=2, sort_keys=True)
                f.write("\n")
            rospy.loginfo("[full_chain] summary_path=%s", os.path.abspath(self.summary_path))

    def _intermediate_goals(self, start: Point, target: Point) -> List[Point]:
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        dz = target[2] - start[2]
        dist = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1.0e-6)
        steps = max(1, int(math.ceil(dist / max(0.2, self.subgoal_step))))
        return [
            (
                start[0] + dx * i / steps,
                start[1] + dy * i / steps,
                start[2] + dz * i / steps,
            )
            for i in range(1, steps + 1)
        ]

    def _run_goal_until_reached(self, goal: Point, radius: float, timeout: float, active_name: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        pub_period = rospy.Duration(1.0 / max(self.goal_publish_rate, 0.1))
        last_pub = rospy.Time(0)
        best = float("inf")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()
            if now - last_pub >= pub_period:
                self._publish_goal(goal)
                last_pub = now
            self._publish_visuals(active_name)
            pos = self._pos()
            if all(math.isfinite(v) for v in pos):
                best = min(best, self._dist(pos, goal))
                if best <= radius:
                    rospy.loginfo("[full_chain] reached subgoal for %s best_distance=%.3f", active_name, best)
                    return True
            rate.sleep()
        rospy.logwarn("[full_chain] subgoal timeout for %s best_distance=%.3f", active_name, best)
        return False

    def _run_competition_waypoint(self, wp: MissionWaypoint) -> bool:
        start = self._pos()
        if not all(math.isfinite(v) for v in start):
            return False
        subgoals = self._intermediate_goals(start, wp.point)
        rospy.loginfo("[full_chain] goal %s via %d planner subgoals -> x=%.3f y=%.3f z=%.3f radius=%.2f",
                      wp.name, len(subgoals), *wp.point, wp.switch_radius)
        for i, subgoal in enumerate(subgoals, start=1):
            radius = min(0.85, max(0.45, self.subgoal_step * 0.65))
            if i == len(subgoals):
                radius = wp.switch_radius
            ok = self._run_goal_until_reached(subgoal, radius, self.subgoal_timeout, wp.name)
            pos = self._pos()
            if all(math.isfinite(v) for v in pos):
                wp.best_distance = min(wp.best_distance, self._dist(pos, wp.point))
                if wp.best_distance <= wp.switch_radius:
                    wp.reached = True
                    rospy.loginfo("[full_chain] reached %s best_distance=%.3f", wp.name, wp.best_distance)
                    return True
            if not ok and i < len(subgoals):
                return False
        pos = self._pos()
        if all(math.isfinite(v) for v in pos):
            wp.best_distance = min(wp.best_distance, self._dist(pos, wp.point))
            wp.reached = wp.best_distance <= wp.switch_radius
        return wp.reached

    def run(self) -> int:
        rospy.logwarn("[full_chain] Nationals full-chain RViz mission: layout cloud -> SUPER -> px4ctrl -> MAVLink UDP -> PX4 SITL iris.")
        code = 9
        try:
            if not self._wait_for(self._connected, 20.0, "/mavros/state connected=True"):
                code = 2
                return code
            if self._armed():
                rospy.logerr("[full_chain] refusing to start: already armed")
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
            self.summary.offboard = self._wait_for(self._offboard, 10.0, "OFFBOARD mode")
            self.summary.armed = self._wait_for(self._armed, 10.0, "armed=True")
            if not (self.summary.offboard and self.summary.armed):
                code = 3
                return code
            self.summary.takeoff = self._wait_for(lambda: abs(self._pos()[2] - self.target_alt) <= 0.30, self.takeoff_timeout, "takeoff hover")
            if not self.summary.takeoff:
                code = 4
                return code

            selected = self._selected_waypoints()
            for wp in selected:
                self.summary.completed_stage = wp.name
                if not self._run_competition_waypoint(wp):
                    rospy.logerr("[full_chain] timeout at %s best_distance=%.3f", wp.name, wp.best_distance)
                    code = 5
                    return code

            self._update_metrics()
            if "/fsm_node" not in self.summary.position_cmd_publishers:
                rospy.logerr("[full_chain] /position_cmd publisher is not /fsm_node: %s", self.summary.position_cmd_publishers)
                code = 6
                return code
            if not any("px4ctrl" in s for s in self.summary.position_cmd_subscribers):
                rospy.logerr("[full_chain] /position_cmd missing px4ctrl subscriber: %s", self.summary.position_cmd_subscribers)
                code = 6
                return code
            if self.summary.cloud_registered_hz < 3.0 or self.summary.odom_hz < 20.0 or self.summary.position_cmd_hz <= 1.0 or self.summary.attitude_hz <= 1.0:
                rospy.logerr("[full_chain] topic rate check failed")
                code = 6
                return code
            code = 0
            return code
        finally:
            self.safe_exit()
            self._write_summary(code)


if __name__ == "__main__":
    rospy.init_node("nationals_full_chain_mission_runner")
    sys.exit(FullChainMissionRunner().run())
