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

The validation script regenerates `mission_planner/data/nationals_seed_2026.txt`
with a switch radius wide enough for SUPER's feasible through-ring target, which
may settle slightly offset from the exact ring center.

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

## Observe Mock Flight In Gazebo

To watch the same mock planning loop inside the generated nationals Gazebo world:

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_gazebo_mock_rviz.sh
```

This is not PX4 SITL and not real vehicle dynamics. The script does not start
MAVROS, px4ctrl, arming, or takeoff. Gazebo only displays the nationals map and a
visual-only `super_mock_drone` model whose pose is synchronized from
`/Odom_high_freq` by `gazebo_mock_drone_pose_bridge.py`.

SUPER still plans from the layout-derived `/cloud_registered`; the point cloud is
not produced by a Gazebo lidar in this stage.

The visualization script writes a temporary waypoint file under `/tmp` with a
slightly wider switch radius so the mock mission continues through the visual
ring targets even when SUPER settles near a feasible point offset from the exact
ring center. The committed validation seed file and
`run_nationals_super_validation.sh` keep their original validation behavior.

If the aircraft moves in RViz but not in Gazebo, check
`/gazebo/set_model_state` and `gazebo_mock_drone_pose_bridge.py`. If Gazebo and
RViz both show odometry but `/position_cmd` is missing, check SUPER and the
published mission goals. If the Gazebo model is static while `/Odom_high_freq`
is absent, check `mock_drone_tracker`.
