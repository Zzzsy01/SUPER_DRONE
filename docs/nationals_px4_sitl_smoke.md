# Nationals PX4 SITL Smoke Workflow

This is a PX4 SITL smoke test, not a real-flight workflow. Do not use it on a real aircraft. It is only meant to verify that PX4 SITL, Gazebo Classic, MAVROS, px4ctrl, SUPER `/position_cmd`, and `/mavros/setpoint_raw/attitude` are connected.

Do not expect a complete ring traversal in this stage. Do not modify SUPER A*, ROG-Map, trajectory optimization, or `quadrotor_msgs/PositionCommand.msg` for this smoke test.

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
make -C ~/PX4-Autopilot px4_sitl_default gazebo-classic_iris
```

It does not start SUPER, px4ctrl, MAVROS, arm, takeoff, land, or publish mission goals.

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

## Terminal C: nationals_sim nodes

If the `nationals_sim` package from `~/ws/gezogo-guosai/gazebo_px4_nationals/nationals_sim` is linked into a sourced catkin workspace, start it with the mission driver disabled:

```bash
roslaunch nationals_sim nationals_nodes.launch \
  layout_file:=$HOME/ws/gezogo-guosai/gazebo_px4_nationals/generated/seed_2026/layout.json \
  start_mission_driver:=false
```

Keeping `start_mission_driver:=false` avoids a second mission source while SUPER is being checked.

## Terminal D-G: Relay, layout cloud, SUPER, mission planner

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_super_sitl_smoke.sh
```

This launch does the following:

- Relays `/mavros/local_position/odom` to `/Odom_high_freq`.
- Publishes `/cloud_registered` from `nationals_layout_cloud_publisher.py`.
- Starts `super_planner` `fsm_node`.
- Starts `super_drone_mission` using `mission_planner/data/nationals_seed_2026.txt`.

If Gazebo does not provide a real `sensor_msgs/PointCloud2` lidar topic, keep using the layout cloud publisher. Real Mid-360S and FAST-LIO are not required for this smoke test.

## Terminal H: px4ctrl

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_nationals_px4ctrl_sitl.sh
```

The wrapper is SITL-only. It tries to pass `no_RC:=true` when `px4ctrl/run_ctrl.launch` declares that arg, and also sets temporary ROS params `/px4ctrl/no_RC=true` and `/no_RC=true`. It does not permanently edit px4ctrl configuration.

Do not arm, take off, or land automatically from this workflow.

## Terminal I: topic smoke check

```bash
cd ~/super_ws
source devel/setup.bash
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
