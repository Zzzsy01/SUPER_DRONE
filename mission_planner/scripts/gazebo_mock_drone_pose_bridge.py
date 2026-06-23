#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry


class GazeboMockDronePoseBridge:
    def __init__(self) -> None:
        self.model_name = rospy.get_param("~model_name", "super_mock_drone")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.update_rate = float(rospy.get_param("~update_rate", 30.0))
        self.latest_odom = None
        self.set_state = None
        rospy.Subscriber("/Odom_high_freq", Odometry, self._odom_cb, queue_size=20)
        rospy.loginfo("[gazebo_mock_drone_pose_bridge] waiting for /gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self._timer_cb)

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _finite_pose(self, msg: Odometry) -> bool:
        vals = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        return all(math.isfinite(v) for v in vals)

    def _timer_cb(self, _event) -> None:
        if self.latest_odom is None or self.set_state is None:
            return
        if not self._finite_pose(self.latest_odom):
            rospy.logwarn_throttle(1.0, "[gazebo_mock_drone_pose_bridge] ignoring non-finite odometry")
            return
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = self.frame_id
        state.pose = self.latest_odom.pose.pose
        state.twist = self.latest_odom.twist.twist
        try:
            self.set_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "[gazebo_mock_drone_pose_bridge] set_model_state failed: %s", exc)


def main() -> None:
    rospy.init_node("gazebo_mock_drone_pose_bridge")
    GazeboMockDronePoseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
