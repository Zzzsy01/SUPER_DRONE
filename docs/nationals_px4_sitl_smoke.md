# Nationals PX4 SITL Smoke Workflow

This is a PX4 SITL smoke test, not a real-flight workflow. Do not use it on a real aircraft. It is only meant to verify that PX4 SITL, Gazebo Classic, MAVROS, px4ctrl, SUPER `/position_cmd`, and `/mavros/setpoint_raw/attitude` are connected.

Do not expect a complete ring traversal in this stage. Do not modify SUPER A*, ROG-Map, trajectory optimization, or `quadrotor_msgs/PositionCommand.msg` for this smoke test.

This workflow is SITL-only. Do not install propellers, do not use it on real hardware, and do not use these scripts for a real flight.

## First Run: Environment Preflight

Run the preflight before opening the multi-terminal smoke workflow:

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/preflight_nationals_px4_sitl_env.sh
```

The preflight uses:

```bash
SUPER_WS=${SUPER_WS:-$HOME/super_ws}
SUPER_DRONE_DIR=${SUPER_DRONE_DIR:-$SUPER_WS/src/SUPER_DRONE}
GEZOGO_DIR=${GEZOGO_DIR:-$HOME/ws/gezogo-guosai}
```

It checks PX4, GeographicLib, `nationals_sim`, `px4ctrl`, generated nationals world/layout files, and catkin package visibility.

If PX4 is not found, install/build PX4 or run with an explicit path:

```bash
PX4_DIR=/path/to/PX4-Autopilot ./scripts/preflight_nationals_px4_sitl_env.sh
PX4_DIR=/path/to/PX4-Autopilot ./scripts/run_nationals_px4_sitl_world.sh
```

If MAVROS reports a GeographicLib error for `egm96-5.pgm`, install the dataset:

```bash
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

or:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

If `nationals_sim` or `px4ctrl` is not found, run the preflight again and check its summary. It will create safe symlinks into `~/super_ws/src` when the source packages are present; it will not overwrite an existing conflicting path.

Type script names exactly as shown. Do not append Chinese punctuation such as `、` after `.sh`.

After preflight passes, use this terminal order:

1. `./scripts/run_nationals_px4_sitl_world.sh`
2. `./scripts/run_nationals_mavros.sh`
3. `roslaunch nationals_sim nationals_nodes.launch layout_file:=${HOME}/ws/gezogo-guosai/gazebo_px4_nationals/generated/seed_2026/layout.json start_mission_driver:=false`
4. `./scripts/run_nationals_super_sitl_smoke.sh`
5. `./scripts/run_nationals_px4ctrl_sitl.sh`
6. `./scripts/check_nationals_px4_sitl_topics.sh`

## Terminal A: PX4 SITL + Gazebo nationals world

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_px4_sitl_world.sh
```

This checks or generates:

```text
~/ws/gezogo-guosai/gazebo_px4_nationals/generated/seed_2026/nationals_field.world
```

Then it exports `PX4_SITL_WORLD` and runs:

```bash
make -C "$PX4_DIR" px4_sitl_default gazebo-classic_iris
```

It searches common PX4 locations and also supports:

```bash
PX4_DIR=/path/to/PX4-Autopilot ./scripts/run_nationals_px4_sitl_world.sh
```

It does not start SUPER, px4ctrl, MAVROS, arm, takeoff, land, target real hardware, or publish mission goals.

## Terminal B: MAVROS

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_mavros.sh
```

This runs:

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Check that `/mavros/state` becomes `connected: True` before continuing.

If it reports a GeographicLib exception for `egm96-5.pgm`, install the dataset as shown in the preflight section.

## Terminal C: nationals_sim nodes

After the preflight has linked `nationals_sim` into `~/super_ws/src` and rebuilt the workspace, start it with the mission driver disabled:

```bash
cd ~/super_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch nationals_sim nationals_nodes.launch \
  layout_file:=${HOME}/ws/gezogo-guosai/gazebo_px4_nationals/generated/seed_2026/layout.json \
  start_mission_driver:=false
```

Keeping `start_mission_driver:=false` avoids a second mission source while SUPER is being checked.

Do not pass `layout_file:=~/ws/...` here. `~` is not expanded inside ROS launch argument values in every context; use `${HOME}` from the shell or an absolute path such as `/home/zsy/ws/...`.

## Terminal D-G: Relay, layout cloud, SUPER, smoke goal

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_super_sitl_smoke.sh
```

This launch does the following:

- Relays `/mavros/local_position/odom` to `/Odom_high_freq`.
- Publishes `/cloud_registered` from `nationals_layout_cloud_publisher.py`.
- Starts `super_planner` `fsm_node`.
- Uses the smoke-only planner config `super_drone_px4_sitl_smoke.yaml`, which keeps the virtual ground below PX4 SITL's disarmed ground odom.
- Publishes one safe smoke-test `/planning/click_goal` above the layout takeoff zone in frame `world`.

By default this smoke launch does not start the full `super_drone_mission` waypoint sequence. It only triggers SUPER planning enough to produce `/position_cmd`. To explicitly test the mission node later, run with `SITL_START_MISSION=true`, but do not use that for this minimal smoke check.

If Gazebo does not provide a real `sensor_msgs/PointCloud2` lidar topic, keep using the layout cloud publisher. Real Mid-360S and FAST-LIO are not required for this smoke test.

## Terminal H: px4ctrl

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_px4ctrl_sitl.sh
```

The wrapper is SITL-only. It tries to pass `no_RC:=true` when `px4ctrl/run_ctrl.launch` declares that arg, and also sets temporary ROS params `/px4ctrl/no_RC=true` and `/no_RC=true`. It does not permanently edit px4ctrl configuration.

Do not arm, take off, land, or target real hardware from this workflow.

## Terminal I: topic smoke check

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/check_nationals_px4_sitl_topics.sh
```

Expected smoke-test signals:

- `/mavros/state` exists and reports `connected: True`.
- `/Odom_high_freq` has frequency from the MAVROS odom relay.
- `/cloud_registered` has frequency from the layout cloud publisher.
- `/position_cmd` has frequency from SUPER.
- `/mavros/setpoint_raw/attitude` has frequency from px4ctrl.
- `/position_cmd` shows a px4ctrl subscriber.
- `rosmsg show quadrotor_msgs/PositionCommand` works.

This is only a control-chain smoke test. It must not be used for real hardware, and it must not be treated as proof that the full nationals mission can pass rings or land.
