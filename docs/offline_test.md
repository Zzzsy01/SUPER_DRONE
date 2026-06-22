# Offline ROS1 Interface Test

This test verifies the SUPER_DRONE ROS1 topic wiring without PX4, FAST-LIO, MID-360 hardware, REAL_DRONE_400 code, or depth camera inputs.

It starts:

- `super_planner/fsm_node`
- `mission_planner/offline_interface_feeder`

The feeder publishes:

- `/Odom_high_freq` as `nav_msgs/Odometry`
- `/cloud_registered` as `sensor_msgs/PointCloud2`
- `/planning/click_goal` as `geometry_msgs/PoseStamped`

The planner should then publish `/position_cmd`.

## Build

From the catkin workspace root:

```bash
cd /home/zsy/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run

Start roscore if it is not already running:

```bash
roscore
```

In another terminal:

```bash
cd /home/zsy/super_ws
source devel/setup.bash
roslaunch mission_planner offline_interface_test.launch
```

Default test values:

- odom frame: `world`
- odom position: `(0, 0, 1)`
- goal: `(2, 0, 1)`
- goal publish delay: `3.0 s`
- point cloud: a few far obstacle points away from the direct short test path

You can override the target:

```bash
roslaunch mission_planner offline_interface_test.launch goal_x:=2.0 goal_y:=1.0 goal_z:=1.0
```

## Check Topics

List the expected topics:

```bash
rostopic list | grep -E 'Odom_high_freq|cloud_registered|planning/click_goal|position_cmd'
```

Check input rates:

```bash
rostopic hz /Odom_high_freq
rostopic hz /cloud_registered
```

Check the test goal:

```bash
rostopic echo -n 1 /planning/click_goal
```

Check planner output:

```bash
rostopic hz /position_cmd
rostopic echo -n 1 /position_cmd
```

Expected result: after the feeder publishes `/planning/click_goal`, `/position_cmd` should appear and publish `quadrotor_msgs/PositionCommand` while the planner is following the generated trajectory.

## Notes

- This is only an offline interface test. It does not validate real-flight dynamics, PX4 control behavior, or FAST-LIO quality.
- This test does not change planner dynamic limits.
- This test does not use `super_drone_mission` or `/traj_start_trigger`; it publishes directly to `/planning/click_goal` to isolate the planner topic interface.
