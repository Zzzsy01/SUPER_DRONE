#!/usr/bin/env python3
import argparse
import json
import math
from typing import Any, Dict, Iterable, Optional, Tuple

import rosbag


Point = Tuple[float, float, float]


def point_from_msg(topic: str, msg: Any) -> Optional[Point]:
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        p = msg.pose.pose.position
        return float(p.x), float(p.y), float(p.z)
    if topic.endswith("position_cmd"):
        p = msg.position
        return float(p.x), float(p.y), float(p.z)
    if topic == "/planning/click_goal":
        p = msg.pose.position
        return float(p.x), float(p.y), float(p.z)
    return None


def vector_from_msg(topic: str, msg: Any, name: str) -> Optional[Point]:
    if not topic.endswith("position_cmd"):
        return None
    value = getattr(msg, name)
    return float(value.x), float(value.y), float(value.z)


def dist(a: Point, b: Point) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def fmt_point(p: Optional[Point]) -> Optional[Tuple[float, float, float]]:
    if p is None:
        return None
    return tuple(round(v, 6) for v in p)


def summarize_topic(bag: rosbag.Bag, topic: str, goal: Optional[Point]) -> Dict[str, Any]:
    count = 0
    first_t = None
    last_t = None
    last_p = None
    last_v = None
    last_a = None
    min_p = [float("inf"), float("inf"), float("inf")]
    max_p = [float("-inf"), float("-inf"), float("-inf")]
    best = None
    for _, msg, stamp in bag.read_messages(topics=[topic]):
        count += 1
        t = stamp.to_sec()
        first_t = t if first_t is None else first_t
        last_t = t
        p = point_from_msg(topic, msg)
        if p is not None:
            last_p = p
            for i, value in enumerate(p):
                min_p[i] = min(min_p[i], value)
                max_p[i] = max(max_p[i], value)
            if goal is not None:
                d = dist(p, goal)
                if best is None or d < best["distance"]:
                    best = {"time": t, "distance": d, "position": p}
        v = vector_from_msg(topic, msg, "velocity")
        a = vector_from_msg(topic, msg, "acceleration")
        if v is not None:
            last_v = v
        if a is not None:
            last_a = a
    return {
        "count": count,
        "first_time": first_t,
        "last_time": last_t,
        "duration": None if first_t is None or last_t is None else last_t - first_t,
        "last_position": fmt_point(last_p),
        "last_velocity": fmt_point(last_v),
        "last_acceleration": fmt_point(last_a),
        "min_position": None if count == 0 or min_p[0] == float("inf") else fmt_point(tuple(min_p)),
        "max_position": None if count == 0 or max_p[0] == float("-inf") else fmt_point(tuple(max_p)),
        "best_to_goal": None if best is None else {
            "time": best["time"],
            "distance": round(best["distance"], 6),
            "position": fmt_point(best["position"]),
        },
    }


def last_cloud_before(bag: rosbag.Bag, cutoff: Optional[float]) -> Dict[str, Any]:
    count = 0
    first_t = None
    last_t = None
    for _, _msg, stamp in bag.read_messages(topics=["/cloud_registered"]):
        t = stamp.to_sec()
        if cutoff is not None and t > cutoff:
            continue
        count += 1
        first_t = t if first_t is None else first_t
        last_t = t
    return {
        "count_before_cutoff": count,
        "first_time": first_t,
        "last_time_before_cutoff": last_t,
        "age_at_cutoff": None if cutoff is None or last_t is None else cutoff - last_t,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze PX4 SITL nationals mission bag timing and command state.")
    parser.add_argument("bag")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-z", type=float)
    args = parser.parse_args()

    goal = None
    if args.goal_x is not None and args.goal_y is not None and args.goal_z is not None:
        goal = (args.goal_x, args.goal_y, args.goal_z)

    topics = [
        "/planning/click_goal",
        "/super_position_cmd",
        "/position_cmd",
        "/Odom_high_freq",
        "/mavros/local_position/odom",
        "/nationals_sim/scoring_odom",
    ]
    with rosbag.Bag(args.bag) as bag:
        summary = {topic: summarize_topic(bag, topic, goal) for topic in topics}
        cutoff = summary["/position_cmd"]["last_time"]
        summary["/cloud_registered"] = last_cloud_before(bag, cutoff)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
