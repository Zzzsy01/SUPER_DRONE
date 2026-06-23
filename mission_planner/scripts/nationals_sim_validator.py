#!/usr/bin/env python3
import math
import os
from typing import List, Tuple

import rospy
import rosgraph
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


Goal = Tuple[float, float, float, float]


class NationalsSimValidator:
    def __init__(self) -> None:
        self.validation_timeout = float(rospy.get_param("~validation_timeout", 120.0))
        self.keep_running = bool(rospy.get_param("~keep_running", False))
        self.waypoints_path = rospy.get_param("~waypoints_path", "")
        self.min_cloud_rate = float(rospy.get_param("~min_cloud_rate", 2.0))
        self.min_odom_rate = float(rospy.get_param("~min_odom_rate", 50.0))
        self.min_cmd_rate = float(rospy.get_param("~min_cmd_rate", 20.0))
        self.goals = self._load_goals(self.waypoints_path)
        self.next_goal = 0
        self.start_time = rospy.Time.now()
        self.first_cloud = None
        self.first_odom = None
        self.first_cmd = None
        self.cloud_count = 0
        self.odom_count = 0
        self.cmd_count = 0
        self.cmd_finite = True
        self.forbidden_takeoff_land = False
        self.forbidden_mavros_attitude = False
        self.finished = False
        self.status_pub = rospy.Publisher("/nationals_sim/validation_status", String, queue_size=1, latch=True)
        rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_cb, queue_size=20)
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=100)
        rospy.Subscriber("/position_cmd", PositionCommand, self._cmd_cb, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(0.2), self._timer_cb)

    def _load_goals(self, path: str) -> List[Goal]:
        goals: List[Goal] = []
        with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                parts = stripped.split()
                if len(parts) != 4:
                    raise RuntimeError(f"invalid waypoint line: {line.rstrip()}")
                x, y, z, switch_dis = [float(v) for v in parts]
                goals.append((x, y, z, max(0.35, switch_dis + 0.15)))
        if len(goals) != 5:
            raise RuntimeError(f"expected 5 nationals waypoints, got {len(goals)} from {path}")
        return goals

    def _count(self, attr_count: str, attr_first: str) -> None:
        if getattr(self, attr_count) == 0:
            setattr(self, attr_first, rospy.Time.now())
        setattr(self, attr_count, getattr(self, attr_count) + 1)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if msg.width * msg.height > 0:
            self._count("cloud_count", "first_cloud")

    def _odom_cb(self, msg: Odometry) -> None:
        self._count("odom_count", "first_odom")
        if self.next_goal >= len(self.goals):
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        gx, gy, gz, radius = self.goals[self.next_goal]
        dist = math.sqrt((x - gx) ** 2 + (y - gy) ** 2 + (z - gz) ** 2)
        if dist <= radius:
            self.next_goal += 1
            rospy.loginfo("[nationals_sim_validator] reached waypoint %d/%d", self.next_goal, len(self.goals))

    def _cmd_cb(self, msg: PositionCommand) -> None:
        self._count("cmd_count", "first_cmd")
        vals = [
            msg.position.x, msg.position.y, msg.position.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z,
            msg.acceleration.x, msg.acceleration.y, msg.acceleration.z,
            msg.yaw,
        ]
        if not all(math.isfinite(v) for v in vals):
            self.cmd_finite = False

    def _topic_has_publishers(self, topic: str) -> bool:
        try:
            master = rosgraph.Master(rospy.get_name())
            pubs, _, _ = master.getSystemState()
            return any(name == topic and len(nodes) > 0 for name, nodes in pubs)
        except Exception:
            return False

    def _rate(self, count: int, first) -> float:
        if count < 2 or first is None:
            return 0.0
        elapsed = (rospy.Time.now() - first).to_sec()
        return float(count - 1) / elapsed if elapsed > 0.0 else 0.0

    def _timer_cb(self, _event) -> None:
        if self.finished:
            return
        self.forbidden_takeoff_land = self.forbidden_takeoff_land or self._topic_has_publishers("/px4ctrl/takeoff_land")
        self.forbidden_mavros_attitude = self.forbidden_mavros_attitude or self._topic_has_publishers("/mavros/setpoint_raw/attitude")
        if self.next_goal >= len(self.goals):
            self._finish(True)
        elif (rospy.Time.now() - self.start_time).to_sec() >= self.validation_timeout:
            self._finish(False)

    def _finish(self, goals_complete: bool) -> None:
        self.finished = True
        cloud_rate = self._rate(self.cloud_count, self.first_cloud)
        odom_rate = self._rate(self.odom_count, self.first_odom)
        cmd_rate = self._rate(self.cmd_count, self.first_cmd)
        cloud_ok = self.cloud_count > 0 and cloud_rate >= self.min_cloud_rate
        odom_ok = self.odom_count > 0 and odom_rate >= self.min_odom_rate
        cmd_ok = self.cmd_count > 0 and cmd_rate >= self.min_cmd_rate
        task_ok = len(self.goals) == 5 and goals_complete
        forbidden_ok = not self.forbidden_takeoff_land and not self.forbidden_mavros_attitude
        passed = cloud_ok and odom_ok and cmd_ok and self.cmd_finite and task_ok and forbidden_ok
        report = (
            f"{'[PASS]' if passed else '[FAIL]'} nationals super validation\n"
            f"  cloud_registered: {'OK' if cloud_ok else 'FAIL'} count={self.cloud_count} rate={cloud_rate:.2f}Hz\n"
            f"  Odom_high_freq: {'OK' if odom_ok else 'FAIL'} count={self.odom_count} rate={odom_rate:.2f}Hz\n"
            f"  position_cmd: {'OK' if cmd_ok else 'FAIL'} count={self.cmd_count} rate={cmd_rate:.2f}Hz\n"
            f"  position_cmd finite: {'OK' if self.cmd_finite else 'FAIL'}\n"
            f"  waypoint_count: {'OK' if len(self.goals) == 5 else 'FAIL'} count={len(self.goals)}\n"
            f"  goal sequence: {'OK' if goals_complete else 'FAIL'} reached={self.next_goal}/{len(self.goals)}\n"
            f"  forbidden flight-control publishers: {'OK' if forbidden_ok else 'FAIL'} "
            f"px4ctrl_takeoff_land={'present' if self.forbidden_takeoff_land else 'absent'} "
            f"mavros_attitude={'present' if self.forbidden_mavros_attitude else 'absent'}\n"
        )
        self.status_pub.publish(String(report))
        print(report, end="", flush=True)
        if not self.keep_running:
            rospy.signal_shutdown("nationals validation finished")


def main() -> None:
    rospy.init_node("nationals_sim_validator")
    NationalsSimValidator()
    rospy.spin()


if __name__ == "__main__":
    main()
