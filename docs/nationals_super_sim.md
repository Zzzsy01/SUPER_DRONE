# Nationals SUPER Mock Simulation

This stage validates the planning software loop for the nationals layout:

`layout.json -> /cloud_registered -> SUPER -> /position_cmd -> mock drone -> validator`

It is not PX4 SITL real flight, does not start MAVROS or px4ctrl, and must not be
used for arming, takeoff, or propeller tests.

## Inputs

The map input is `layout.json` from `gezogo-guosai`. The point cloud is generated
directly from structured layout geometry by:

`mission_planner/scripts/nationals_layout_cloud_publisher.py`

This means `/cloud_registered` is not coming from a real Gazebo lidar in this
stage. The node publishes `sensor_msgs/PointCloud2` at a configurable frame,
rate, and point density.

## Waypoints

Generate the mission file from the layout:

```bash
cd ~/super_ws/src/SUPER_DRONE
python3 mission_planner/scripts/generate_nationals_waypoints.py \
  --layout ~/ws/gezogo-guosai/layout.json \
  --output mission_planner/data/nationals_seed_2026.txt
```

The file format is:

```text
x y z switch_dis
```

The first four waypoints are scoring ring targets. The final waypoint is the
pre-landing hover point.

## One-command Validation

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_super_validation.sh
```

The script builds the workspace, generates waypoints, launches
`nationals_super_mock.launch`, waits for the validator, and tears down roslaunch.

## Scope Boundary

Passing this validation only proves the SUPER planning loop against the nationals
layout-derived point cloud. The next stage is PX4 SITL + MAVROS + px4ctrl.

Before PX4 SITL, confirm whether the Gazebo vehicle publishes a usable
`sensor_msgs/PointCloud2`. If it does not, either add a lidar plugin in Gazebo or
continue using `nationals_layout_cloud_publisher.py` as the temporary planning
input.
