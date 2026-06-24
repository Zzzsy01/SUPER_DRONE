#!/usr/bin/env python3
import math
import sys
from dataclasses import dataclass
from typing import Optional

import rospy
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand


@dataclass
class SmokeResult:
    offboard: bool = False
    armed: bool = False
    reached_hover: bool = False
    hover_stable: bool = False
    safe_exit: bool = False


class HoverSmoke:
    def __init__(self) -> None:
        self.target_alt = float(rospy.get_param("~target_alt", 1.0))
        self.hover_seconds = float(rospy.get_param("~hover_seconds", 8.0))
        self.prestream_seconds = float(rospy.get_param("~prestream_seconds", 3.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 25.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 25.0))
        self.rate_hz = float(rospy.get_param("~cmd_rate", 50.0))
        self.hover_z_tolerance = float(rospy.get_param("~hover_z_tolerance", 0.25))
        self.max_alt = float(rospy.get_param("~max_alt", 1.8))
        self.max_xy_drift = float(rospy.get_param("~max_xy_drift", 2.0))

        self.state: Optional[State] = None
        self.odom: Optional[Odometry] = None
        self.last_odom_time = rospy.Time(0)
        self.last_attitude_time = rospy.Time(0)

        self.result = SmokeResult()
        self.takeoff_land_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.cmd_pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=20)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self._attitude_cb, queue_size=20)

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = rospy.Time.now()

    def _attitude_cb(self, _msg: AttitudeTarget) -> None:
        self.last_attitude_time = rospy.Time.now()

    def _connected(self) -> bool:
        return bool(self.state and self.state.connected)

    def _armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _offboard(self) -> bool:
        return bool(self.state and self.state.mode == "OFFBOARD")

    def _z(self) -> float:
        if self.odom is None:
            return float("nan")
        return float(self.odom.pose.pose.position.z)

    def _xy(self) -> tuple:
        if self.odom is None:
            return (float("nan"), float("nan"))
        p = self.odom.pose.pose.position
        return float(p.x), float(p.y)

    def _odom_fresh(self) -> bool:
        return self.odom is not None and (rospy.Time.now() - self.last_odom_time).to_sec() < 1.0

    def _attitude_fresh(self) -> bool:
        return (rospy.Time.now() - self.last_attitude_time).to_sec() < 0.5

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                rospy.loginfo("[hover_smoke] %s: OK", label)
                return True
            rate.sleep()
        rospy.logerr("[hover_smoke] %s: TIMEOUT", label)
        return False

    def _publish_takeoff_land(self, command: int, seconds: float = 0.5) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.takeoff_land_pub.publish(msg)
            rate.sleep()

    def _position_cmd(self, x: float, y: float, z: float, yaw: float = 0.0) -> PositionCommand:
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        msg.yaw_dot = 0.0
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        msg.kx = [2.0, 2.0, 2.0]
        msg.kv = [1.5, 1.5, 1.5]
        return msg

    def _publish_hover_cmds(self, x: float, y: float, z: float, duration: float) -> None:
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.cmd_pub.publish(self._position_cmd(x, y, z))
            rate.sleep()

    def _abort_reason(self, start_xy: tuple) -> Optional[str]:
        if not self._connected():
            return "mavros disconnected"
        if not self._odom_fresh():
            return "odom timeout"
        if not self._attitude_fresh():
            return "attitude setpoint timeout"
        z = self._z()
        if math.isfinite(z) and z > self.max_alt:
            return f"altitude too high: {z:.2f}m"
        x, y = self._xy()
        if all(math.isfinite(v) for v in (x, y, start_xy[0], start_xy[1])):
            drift = math.hypot(x - start_xy[0], y - start_xy[1])
            if drift > self.max_xy_drift:
                return f"xy drift too high: {drift:.2f}m"
        return None

    def _plain_disarm(self) -> bool:
        try:
            resp = self.arm_srv(False)
            return bool(resp.success)
        except rospy.ServiceException as exc:
            rospy.logwarn("[hover_smoke] plain disarm service failed: %s", exc)
            return False

    def _force_disarm(self) -> bool:
        try:
            resp = self.command_srv(
                broadcast=False,
                command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
                confirmation=0,
                param1=0.0,
                param2=21196.0,
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            return bool(resp.success)
        except rospy.ServiceException as exc:
            rospy.logerr("[hover_smoke] force disarm service failed: %s", exc)
            return False

    def _wait_disarmed(self, timeout: float) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                return True
            rate.sleep()
        return not self._armed()

    def safe_exit(self) -> None:
        rospy.loginfo("[hover_smoke] stopping /position_cmd before LAND")
        rospy.sleep(1.0)
        rospy.loginfo("[hover_smoke] publishing LAND command")
        self._publish_takeoff_land(TakeoffLand.LAND, seconds=1.0)
        deadline = rospy.Time.now() + rospy.Duration(self.land_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self._armed():
                self.result.safe_exit = True
                rospy.loginfo("[hover_smoke] disarmed after LAND")
                return
            if self._odom_fresh() and self._z() < 0.18:
                break
            rate.sleep()

        if not self._armed():
            self.result.safe_exit = True
            return

        if not (self._odom_fresh() and self._z() < 0.25):
            rospy.logerr("[hover_smoke] refusing force disarm: vehicle is not confirmed near ground")
            self.result.safe_exit = False
            return

        rospy.logwarn("[hover_smoke] attempting disarm after LAND")
        disarm_deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and rospy.Time.now() < disarm_deadline:
            if self._plain_disarm() and self._wait_disarmed(1.0):
                self.result.safe_exit = True
                rospy.loginfo("[hover_smoke] disarmed after LAND")
                return
            rate.sleep()

        rospy.logwarn("[hover_smoke] plain disarm rejected near ground; using SITL force disarm")
        self.result.safe_exit = self._force_disarm() and self._wait_disarmed(3.0)

    def run(self) -> int:
        rospy.logwarn("[hover_smoke] SITL ONLY. Do not use on real hardware.")
        if not self._wait_for(self._connected, 10.0, "/mavros/state connected=True"):
            return 2
        if self._armed():
            rospy.logerr("[hover_smoke] refusing to start: vehicle is already armed")
            return 2
        if not self._wait_for(self._odom_fresh, 10.0, "/mavros/local_position/odom fresh"):
            return 2
        if not self._wait_for(self._attitude_fresh, self.prestream_seconds, "pre-OFFBOARD attitude setpoints"):
            return 2

        start_xy = self._xy()
        rospy.loginfo("[hover_smoke] initial position x=%.3f y=%.3f z=%.3f", start_xy[0], start_xy[1], self._z())
        rospy.loginfo("[hover_smoke] publishing TAKEOFF command")
        self._publish_takeoff_land(TakeoffLand.TAKEOFF, seconds=1.0)

        try:
            self.result.offboard = self._wait_for(self._offboard, 8.0, "OFFBOARD mode")
            self.result.armed = self._wait_for(self._armed, 8.0, "armed=True")
            if not (self.result.offboard and self.result.armed):
                return 3

            deadline = rospy.Time.now() + rospy.Duration(self.takeoff_timeout)
            rate = rospy.Rate(20)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                reason = self._abort_reason(start_xy)
                if reason:
                    rospy.logerr("[hover_smoke] abort during takeoff: %s", reason)
                    return 4
                if abs(self._z() - self.target_alt) <= self.hover_z_tolerance:
                    self.result.reached_hover = True
                    break
                rate.sleep()
            if not self.result.reached_hover:
                rospy.logerr("[hover_smoke] did not reach %.2fm hover; current z=%.3f", self.target_alt, self._z())
                return 4

            hover_x, hover_y = self._xy()
            rospy.loginfo("[hover_smoke] reached hover near z=%.3f; publishing hold /position_cmd", self._z())
            max_z_error = 0.0
            max_xy = 0.0
            hover_deadline = rospy.Time.now() + rospy.Duration(self.hover_seconds)
            cmd_rate = rospy.Rate(self.rate_hz)
            while not rospy.is_shutdown() and rospy.Time.now() < hover_deadline:
                reason = self._abort_reason((hover_x, hover_y))
                if reason:
                    rospy.logerr("[hover_smoke] abort during hover: %s", reason)
                    return 5
                self.cmd_pub.publish(self._position_cmd(hover_x, hover_y, self.target_alt))
                z = self._z()
                x, y = self._xy()
                max_z_error = max(max_z_error, abs(z - self.target_alt))
                max_xy = max(max_xy, math.hypot(x - hover_x, y - hover_y))
                cmd_rate.sleep()
            self.result.hover_stable = max_z_error <= self.hover_z_tolerance and max_xy <= 0.5
            rospy.loginfo("[hover_smoke] hover max_z_error=%.3f max_xy_drift=%.3f", max_z_error, max_xy)
            return 0 if self.result.hover_stable else 5
        finally:
            self.safe_exit()
            rospy.loginfo(
                "[hover_smoke] summary offboard=%s armed=%s reached_hover=%s hover_stable=%s safe_exit=%s",
                self.result.offboard,
                self.result.armed,
                self.result.reached_hover,
                self.result.hover_stable,
                self.result.safe_exit,
            )


if __name__ == "__main__":
    rospy.init_node("nationals_px4_sitl_hover_smoke")
    sys.exit(HoverSmoke().run())
