# Gazebo Obstacle Course Validation

`competition_sim.launch` and `gazebo_planning_sim.launch` are smoke tests for the ROS interface loop. They verify point cloud, odom, goal, `/position_cmd`, and mock tracking, but the default route is intentionally easy.

`gazebo_obstacle_course.launch` is the explicit obstacle-avoidance test. It places a real Gazebo box obstacle at x=4 that blocks the straight line from the start near `(0, 0, 1.0)` to the goal `(8, 0, 1.0)`. The `/cloud_registered` fallback cloud is generated from the same obstacle geometry, so RViz and Gazebo should agree.

## Automatic Validation

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/run_obstacle_course_validation.sh
```

The script builds the workspace, starts a private ROS master, runs Gazebo headless, publishes the default goal, and waits for the validator. A passing run ends with:

```text
[PASS] obstacle course validation
PASS: obstacle course validation succeeded
```

## Manual Gazebo and RViz

```bash
cd ~/super_ws
source devel/setup.bash
roslaunch mission_planner gazebo_obstacle_course.launch gui:=true rviz:=true auto_goal:=false
```

Then publish a goal manually:

```bash
cd ~/super_ws/src/SUPER_DRONE
./scripts/pub_planning_goal.sh 8.0 0.0 1.0
```

Use `auto_goal:=true` to let the launch file publish the default goal after a short delay.

## Normal Gazebo View

- A red obstacle wall is visible near x=4, spanning roughly `y=-1.0` to `y=1.0`.
- The start pad is near `(0, 0)` and the goal pad is near `(8, 0)`.
- The mock drone starts above the takeoff area and should move around the obstacle instead of passing through it.
- No PX4, MAVROS, or px4ctrl nodes are started.

## Normal RViz View

- Fixed Frame is `world`.
- `/cloud_registered` overlaps the red Gazebo obstacle.
- `/Odom_high_freq` shows the mock drone moving.
- `/planning/click_goal` shows the active goal.
- The planned path or trajectory bends toward positive or negative y before returning to the goal.
- `/obstacle_course/validation_status` reports PASS when the goal is reached without entering the obstacle volume.

## PASS/FAIL Checks

The validator subscribes to `/Odom_high_freq`, `/position_cmd`, `/cloud_registered`, and `/planning/click_goal`.

- `/position_cmd` rate must be greater than 20 Hz.
- `/position_cmd` position, velocity, acceleration, and yaw must be finite.
- The goal must be reached within 0.5 m.
- Odom must not enter the configured obstacle box plus safety margin.
- The odom history must reach `max |y| > 1.2 m`, proving it did not follow the blocked straight line.

## Common Failures

- No middle obstacle in Gazebo: confirm `mission_planner/worlds/obstacle_course.world` is loaded.
- Point cloud and Gazebo obstacle do not align: check `/cloud_registered` and `/competition_sim/field_markers` in RViz.
- Trajectory goes through the obstacle: inspect `/cloud_registered`, map parameters, and validator output.
- Goal does not arrive: check that `/planning/click_goal` was published in frame `world`.
- No `/position_cmd`: check `fsm_node`, `/Odom_high_freq`, and `/cloud_registered`.
- `nan` or `inf`: inspect the first bad `/position_cmd` message and planner logs.
