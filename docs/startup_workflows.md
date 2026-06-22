# Startup Workflows

## Gazebo-lite Manual Startup

```bash
cd ~/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch mission_planner gazebo_planning_sim.launch
```

Normal:

- Gazebo opens `competition_smoke.world`.
- RViz fixed frame is `world`.
- `/cloud_registered`, `/Odom_high_freq`, `/planning/click_goal`, `/position_cmd`, and `/mock_drone/status` are active.
- The mock drone moves through the goal sequence.

Failure:

- No Gazebo window: check `gazebo_ros` installation.
- No point cloud: check `gazebo_cloud_bridge` warnings.
- No `/position_cmd`: check odom, cloud, and goal topics.

## Gazebo-lite tmux Startup

```bash
./scripts/tmux_gazebo_planning.sh
```

This opens session `gazebo_planning` with Gazebo/RViz and topic monitors. It does not start px4ctrl, MAVROS, PX4 SITL, takeoff, landing, or flight-control publishers.

Stop only the managed sessions:

```bash
./scripts/kill_tmux_sessions.sh
```

## Gazebo-lite Automatic Validation

```bash
./scripts/run_gazebo_planning_validation.sh
```

Normal end:

```text
PASS: gazebo planning validation succeeded
```

Logs are saved under `logs/gazebo_planning_<timestamp>.log` and are ignored by Git.

## Real No-Prop Manual Connection Check

Use separate terminals as documented in:

```text
docs/real_flight_startup.md
```

Do not install propellers. Do not arm. Do not take off.

Minimum checks:

```bash
rostopic echo -n 1 /mavros/state
./scripts/check_mid360s_topics.sh
rostopic hz /cloud_registered
rostopic hz /Odom_high_freq
rostopic info /position_cmd
```

## Real No-Prop tmux Startup

```bash
./scripts/tmux_real_check.sh
```

Mid-360S and FAST-LIO commands are configurable:

```bash
MID360S_START_CMD="./start_lidar_mid360s.sh" \
FAST_LIO_LAUNCH_CMD="roslaunch fast_lio mapping_mid360s.launch" \
./scripts/tmux_real_check.sh
```

The script does not run takeoff, landing, arming, or `/px4ctrl/takeoff_land` commands. The rosbag pane waits for manual Enter before recording.

## Mid-360S Topic Check

```bash
./scripts/check_mid360s_topics.sh
```

Override raw topic names:

```bash
MID360S_POINTS_TOPIC=/livox/lidar/pointcloud \
MID360S_IMU_TOPIC=/livox/imu \
./scripts/check_mid360s_topics.sh
```

Normal:

- Mid-360S raw point topic exists and has rate.
- `/cloud_registered` exists and has rate.
- `/Odom_high_freq` exists and has rate.

Failure:

- `Mid-360S driver topic missing`: check Mid-360S driver launch/config.
- `FAST-LIO cloud_registered missing`: check FAST-LIO input topic remap.
- `FAST-LIO odom missing`: check FAST-LIO status.

## Real Formal Flight Startup

Use `docs/real_flight_startup.md`.

Formal flow:

- REAL_DRONE_400 starts Mid-360S driver, MAVROS, FAST-LIO, and px4ctrl.
- SUPER starts only:
  ```bash
  roslaunch mission_planner real_competition.launch
  ```
- SUPER RViz starts only:
  ```bash
  roslaunch mission_planner real_competition_rviz.launch
  ```
- Takeoff and landing scripts are run manually only after safety approval.

## Workflow Boundaries

- `run_competition_sim_validation.sh`: previous closed-loop mock validation only.
- `run_gazebo_planning_validation.sh`: Gazebo-lite planning validation only.
- `tmux_real_check.sh`: no-prop connection check window layout only.
- None of these scripts approve powered flight.
