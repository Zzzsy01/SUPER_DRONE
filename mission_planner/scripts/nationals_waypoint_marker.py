#!/usr/bin/env python3
import os
from typing import List, Tuple

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


Waypoint = Tuple[float, float, float, float]


def load_waypoints(path: str) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    with open(os.path.expanduser(path), "r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) != 4:
                raise RuntimeError(f"invalid waypoint line: {line.rstrip()}")
            waypoints.append(tuple(float(v) for v in parts))
    return waypoints


def color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


def point(x: float, y: float, z: float) -> Point:
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = z
    return msg


class NationalsWaypointMarker:
    def __init__(self) -> None:
        self.path = rospy.get_param("~waypoints_path", "")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.waypoints = load_waypoints(self.path)
        self.pub = rospy.Publisher("/nationals/waypoints", MarkerArray, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._timer_cb)
        rospy.loginfo("[nationals_waypoint_marker] loaded %d waypoints from %s", len(self.waypoints), self.path)

    def _base_marker(self, marker_id: int, marker_type: int, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _timer_cb(self, _event) -> None:
        arr = MarkerArray()
        line = self._base_marker(0, Marker.LINE_STRIP, "nationals_waypoint_path")
        line.scale.x = 0.04
        line.color = color(0.1, 0.7, 1.0, 0.9)
        for x, y, z, _switch_dis in self.waypoints:
            line.points.append(point(x, y, z))
        arr.markers.append(line)

        for i, (x, y, z, switch_dis) in enumerate(self.waypoints):
            sphere = self._base_marker(i, Marker.SPHERE, "nationals_waypoint_points")
            sphere.pose.position = point(x, y, z)
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color = color(1.0, 0.75, 0.05, 0.95)
            arr.markers.append(sphere)

            ring = self._base_marker(i, Marker.CYLINDER, "nationals_waypoint_switch_dis")
            ring.pose.position = point(x, y, z)
            ring.scale.x = switch_dis * 2.0
            ring.scale.y = switch_dis * 2.0
            ring.scale.z = 0.025
            ring.color = color(0.2, 1.0, 0.25, 0.22)
            arr.markers.append(ring)

            text = self._base_marker(i, Marker.TEXT_VIEW_FACING, "nationals_waypoint_labels")
            text.pose.position = point(x, y, z + 0.35)
            text.scale.z = 0.32
            text.color = color(1.0, 1.0, 1.0, 1.0)
            text.text = str(i)
            arr.markers.append(text)
        self.pub.publish(arr)


def main() -> None:
    rospy.init_node("nationals_waypoint_marker")
    NationalsWaypointMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
