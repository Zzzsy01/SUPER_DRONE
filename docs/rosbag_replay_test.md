# ROS Bag Replay Interface Test

This test verifies the SUPER_DRONE planner chain with recorded ROS1 data.

The launch file starts only:

- `super_planner/fsm_node`
- `mission_planner/goal_feeder`

It does not start `offline_interface_feeder`, `px4ctrl`, or any node that publishes `/px4ctrl/takeoff_land`.

The tested bag path is:

```bash
/home/zsy/super_ws/test_bags/f3.bag
```

Do not commit bag files to GitHub. Keep `*.bag` files outside the repository or ignored by Git.

## Check the Bag

Inspect duration, message types, and topics:

```bash
source /opt/ros/noetic/setup.bash
rosbag info /home/zsy/super_ws/test_bags/f3.bag
```

For `f3.bag`, the required planner input topics are already present:

- `/Odom_high_freq`: `nav_msgs/Odometry`
- `/cloud_registered`: `sensor_msgs/PointCloud2`

## Build

From the catkin workspace root:

```bash
cd /home/zsy/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Start the Planner Chain

Terminal 1:

```bash
cd /home/zsy/super_ws
source devel/setup.bash
roslaunch mission_planner rosbag_replay_test.launch
```

The launch sets `/use_sim_time=true`. The goal feeder publishes one `/planning/click_goal` after `goal_delay` seconds of bag `/clock` time.

Override the test goal if needed:

```bash
roslaunch mission_planner rosbag_replay_test.launch goal_x:=2.0 goal_y:=1.0 goal_z:=1.0 goal_delay:=5.0
```

## Play the Bag

Terminal 2:

```bash
cd /home/zsy/super_ws
source devel/setup.bash
rosbag play --clock --pause /home/zsy/super_ws/test_bags/f3.bag
```

Press `space` in the `rosbag play` terminal to unpause. Keep `--clock`; the planner launch uses simulated time.

## Remap Other Bag Topics

If another bag uses different topic names, inspect it first:

```bash
rosbag info /path/to/other.bag
```

Then remap the recorded topics to the SUPER_DRONE interface while playing:

```bash
rosbag play --clock --pause /path/to/other.bag \
  /recorded_odom_topic:=/Odom_high_freq \
  /recorded_cloud_topic:=/cloud_registered
```

The target planner topics expected by `super_planner/config/super_drone_ros1.yaml` are:

- `/Odom_high_freq`
- `/cloud_registered`

## Check Planner Output

Confirm topics are present:

```bash
rostopic list | grep -E 'Odom_high_freq|cloud_registered|planning/click_goal|position_cmd'
```

Check input rates:

```bash
rostopic hz /Odom_high_freq
rostopic hz /cloud_registered
```

Check the goal feeder output:

```bash
rostopic echo -n 1 /planning/click_goal
```

Check planner command output:

```bash
rostopic hz /position_cmd
rostopic echo -n 1 /position_cmd
```

Expected result: after `/planning/click_goal` is published and the bag provides fresh odom and point cloud data, `/position_cmd` should publish `quadrotor_msgs/PositionCommand`.

## Check Frame IDs

Check odometry frame:

```bash
rostopic echo -n 1 /Odom_high_freq/header
```

Check point cloud frame:

```bash
rostopic echo -n 1 /cloud_registered/header
```

The frame should be consistent with the planner config, typically `world` or `map`. For this repository, `super_planner/config/super_drone_ros1.yaml` visualizes in `world`.

## Check for NaN or Inf in /position_cmd

Sample `/position_cmd` to CSV:

```bash
timeout 10 rostopic echo -p /position_cmd > /tmp/position_cmd.csv
```

Search for non-finite values:

```bash
awk -F, 'NR > 1 { for (i = 1; i <= NF; i++) if ($i ~ /nan|inf/i) { print; bad = 1; exit 1 } } END { if (!bad) print "finite sample" }' /tmp/position_cmd.csv
```

If the command prints `finite sample`, the sampled `/position_cmd` numeric fields did not contain `nan` or `inf`.
