# Real Flight Startup Notes

This document is for real-flight preparation only. It is not an automatic flight script.

Important correction: the real lidar is **Livox Mid-360S**, not ordinary Mid-360.

Mid-360 and Mid-360S driver launch files may differ. Historical filenames such as `mapping_mid360.launch` or `start_lidar.sh` may still exist in `REAL_DRONE_400`; do not assume those names prove Mid-360S support. Confirm that the selected Mid-360S driver publishes the expected raw lidar and IMU topics before starting flight checks.

The simulation scripts are not real-flight scripts:

- `scripts/run_competition_sim_validation.sh` is mock closed-loop validation only.
- `scripts/run_gazebo_planning_validation.sh` is Gazebo-lite validation only.

For real flight, use the REAL_DRONE_400 Mid-360S driver, FAST-LIO, px4ctrl, MAVROS, and takeoff/land workflow. SUPER should only start `real_competition.launch`; do not start mock drone, Gazebo, or simulation validators.

## Terminal 0: Mid-360S Lidar

```bash
cd ~/REAL_DRONE_400
source devel/setup.bash
./start_lidar.sh
```

If Mid-360S uses a different driver launch file, replace the command with the manually confirmed Mid-360S-specific command.

## Terminal 1: MAVROS

```bash
cd ~/REAL_DRONE_400
source devel/setup.bash
roslaunch mavros px4.launch
```

## Terminal 2: FAST-LIO

```bash
cd ~/REAL_DRONE_400
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

`mapping_mid360.launch` may be a historical name. If Mid-360S uses a different launch/config, replace it with the Mid-360S-specific launch.

## Terminal 3: px4ctrl

```bash
cd ~/REAL_DRONE_400
source devel/setup.bash
roslaunch px4ctrl run_ctrl.launch
```

## Terminal 4: SUPER Real Planner

```bash
cd ~/super_ws
source devel/setup.bash
roslaunch mission_planner real_competition.launch
```

This starts SUPER planner and the competition mission layer only.

## Terminal 5: RViz

```bash
cd ~/super_ws
source devel/setup.bash
roslaunch mission_planner real_competition_rviz.launch
```

## Terminal 6: Rosbag Record

```bash
rosbag record -O real_check.bag --lz4 \
  /Odom_high_freq \
  /cloud_registered \
  /position_cmd \
  /planning/click_goal \
  /mavros/state \
  /mavros/local_position/odom \
  /mavros/setpoint_raw/attitude
```

Do not commit bag files to GitHub.

## Takeoff and Landing

Run these only after manual approval and safety confirmation.

Takeoff:

```bash
bash home_shfiles/takeoff.sh
```

Landing:

```bash
bash home_shfiles/land.sh
```

## No-Prop Check Order

Do this before any powered flight:

1. Do not install propellers.
2. Check MAVROS connected.
3. Check `/mavros/state`.
4. Check Mid-360S raw topic.
5. Check `/cloud_registered`.
6. Check `/Odom_high_freq`.
7. Check whether px4ctrl subscribes to `/position_cmd`.
8. Check whether SUPER publishes `/position_cmd`.
9. Do not arm, do not take off, and do not run takeoff scripts until a human confirms the full chain.

Useful topic checks:

```bash
./scripts/check_mid360s_topics.sh
rostopic info /position_cmd
rostopic echo -n 1 /mavros/state
```

Override Mid-360S topic names for checking:

```bash
MID360S_POINTS_TOPIC=/your/mid360s/points \
MID360S_IMU_TOPIC=/your/mid360s/imu \
./scripts/check_mid360s_topics.sh
```
