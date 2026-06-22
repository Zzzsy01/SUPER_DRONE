# Competition Sim Validation

This stage validates the ROS1 software closed loop:

```text
competition point-cloud map -> SUPER planner -> /position_cmd
-> mock_drone_tracker -> /Odom_high_freq -> SUPER replanning
```

It does not validate real PX4 dynamics, motor response, MAVROS behavior, controller gains, or flight safety. It also does not start `px4ctrl`, MAVROS, rosbag playback, or any real flight-control command publisher.

## One-Command Run

From the repository root:

```bash
./scripts/run_competition_sim_validation.sh
```

The script runs:

```bash
cd ~/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch mission_planner competition_sim.launch rviz:=false
```

Logs are saved under:

```bash
logs/competition_sim_<timestamp>.log
```

`logs/` is ignored by Git.

## Manual Run with RViz

```bash
cd ~/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch mission_planner competition_sim.launch
```

The default launch opens RViz. For terminal-only validation:

```bash
roslaunch mission_planner competition_sim.launch rviz:=false
```

## Normal Terminal Output

Expected behavior:

- `competition_map_publisher` reports generated map points.
- `competition_goal_sequence` publishes goal 1 through goal 5.
- `mock_drone_tracker` publishes `/Odom_high_freq` and `/mock_drone/status`.
- `fsm_node` plans and publishes `/position_cmd`.
- `sim_validator` prints a final block beginning with `[PASS]`.

A successful script run ends with:

```text
PASS: competition sim validation succeeded
```

## Normal RViz Output

RViz fixed frame is `world`.

Expected displays:

- `/cloud_registered`: simplified competition obstacle point cloud.
- `/competition_map/markers`: field boundary, takeoff zone, structured boxes, trees, static moving-obstacle placeholders, and gate.
- `/Odom_high_freq`: mock drone odometry arrow moving through the course.
- `/fsm_node/fsm/path`: executed planner path.
- `/competition_sim/current_goal`: current goal sphere.

## PASS/FAIL Checks

`sim_validator` checks:

- `/cloud_registered` exists and is above 2 Hz.
- `/Odom_high_freq` exists and is above 50 Hz.
- `/planning/click_goal` is published.
- `/position_cmd` exists and is above 20 Hz.
- `/mock_drone/status` is published.
- `/position_cmd` position, velocity, acceleration, and yaw are finite.
- No publisher exists on `/px4ctrl/takeoff_land`.
- No publisher exists on `/mavros/setpoint_raw/attitude`.
- The mock drone approaches all default goals in sequence.

## Troubleshooting

No point cloud:

```bash
rostopic hz /cloud_registered
rostopic echo -n 1 /cloud_registered/header
rostopic echo -n 1 /competition_map/markers
```

Check that `competition_map_publisher` is running:

```bash
rosnode list | grep competition_map_publisher
```

No odom:

```bash
rostopic hz /Odom_high_freq
rostopic echo -n 1 /mock_drone/status
rosnode list | grep mock_drone_tracker
```

No `/position_cmd`:

```bash
rostopic hz /position_cmd
rostopic echo -n 1 /planning/click_goal
rostopic hz /cloud_registered
rostopic hz /Odom_high_freq
```

Also inspect the `fsm_node` terminal output for `No odom`, invalid goal, or planning failures.

Trajectory appears to cross obstacles:

- Confirm RViz fixed frame is `world`.
- Check whether `/cloud_registered` and `/competition_map/markers` overlap.
- Reduce obstacle counts or increase goal clearance through launch args.
- This stage uses a simplified point-cloud map; it is for software interface closure, not final course tuning.

Goal does not switch:

```bash
rostopic echo -n 1 /competition_sim/current_goal
rostopic echo -n 1 /Odom_high_freq
rostopic echo -n 1 /mock_drone/status
```

The goal sequence switches after the mock drone enters `reach_radius` and waits `goal_switch_delay`.

NaN or Inf in command:

```bash
timeout 10 rostopic echo -p /position_cmd > /tmp/position_cmd.csv
awk -F, 'NR > 1 { for (i = 1; i <= NF; i++) if ($i ~ /nan|inf/i) { print; bad = 1; exit 1 } } END { if (!bad) print "finite sample" }' /tmp/position_cmd.csv
```

Misstarted `px4ctrl` or MAVROS:

```bash
rostopic info /px4ctrl/takeoff_land
rostopic info /mavros/setpoint_raw/attitude
```

Any publisher on either topic is a validation failure for this stage.

## Useful Launch Overrides

Run without RViz:

```bash
roslaunch mission_planner competition_sim.launch rviz:=false
```

Adjust map size:

```bash
roslaunch mission_planner competition_sim.launch field_length:=12.0 field_width:=8.0 field_height:=2.0
```

Adjust mock drone limits:

```bash
roslaunch mission_planner competition_sim.launch mock_max_vel:=1.2 mock_max_acc:=0.8
```

Adjust goals:

```bash
roslaunch mission_planner competition_sim.launch goal1_x:=2.0 goal1_y:=-1.5 final_x:=10.0 final_y:=0.0
```

## Next Stage

After this validation passes, the next step is a no-prop PX4/MAVROS connection check. Do not treat this mock simulation as approval for powered flight.
