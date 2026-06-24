# Full Teammate Repository Integration

Branch: `dev/nationals-px4-sitl-smoke`

Teammate remote: `teammate/main` from `https://github.com/kkkkkai-pro/SUPER_DRONE.git`

This integration was done manually by selecting paths from `teammate/main`. It did not use `git merge teammate/main` and did not overwrite the full repository.

## Comparison Summary

The current branch and `teammate/main` share a common base but have diverged in different directions.

Current branch contributions kept:

- Obstacle course validation workflow.
- Nationals mock and Gazebo mock workflows.
- Nationals layout generation, layout-cloud publisher, waypoint marker, and in-bounds validation.
- PX4 SITL smoke scripts and environment preflight.
- Documentation for simulation, validation, startup, and PX4 SITL smoke.
- Existing SUPER planner, ROG-Map, trajectory optimization, and `PositionCommand.msg` behavior.

Teammate repository contributions selected:

- `realflight_modules/px4ctrl`
- `realflight_modules/mid360_fastlio`
- `realflight_modules/realsense-ros`
- `utils/uav_utils`
- `utils/cmake_utils`
- `mission_planner/launch/super_drone_realflight.launch`
- `quadrotor_msgs/Px4ctrlDebug.msg` support needed by `px4ctrl`

## Integration Classes

### A. Kept From Current Branch

- `scripts/run_obstacle_course_validation.sh`
- `scripts/run_nationals_super_validation.sh`
- `scripts/preflight_nationals_px4_sitl_env.sh`
- `scripts/run_nationals_*`
- `scripts/check_nationals_px4_sitl_topics.sh`
- Existing `docs/nationals_*`, validation, startup, and rosbag docs
- `mission_planner/Apps/*validator*`
- `mission_planner/Apps/*mock*`
- `mission_planner/launch/*nationals*`
- `mission_planner/launch/gazebo_obstacle_course.launch`
- `mission_planner/launch/gazebo_planning_sim.launch`
- `mission_planner/launch/competition_sim.launch`
- `mission_planner/scripts/nationals_*`
- `mission_planner/rviz/*`
- `mission_planner/worlds/*`
- `mission_planner/models/super_mock_drone/*`
- `mars_uav_sim/mars_quadrotor_msgs/msg/PositionCommand.msg`

### B. Adopted From Teammate

- `realflight_modules/px4ctrl`
- `utils/uav_utils`
- `utils/cmake_utils`
- `realflight_modules/mid360_fastlio`
- `realflight_modules/realsense-ros`
- `mission_planner/launch/super_drone_realflight.launch`

### C. Manually Merged

- `mars_uav_sim/mars_quadrotor_msgs/CMakeLists.txt`
- `mars_uav_sim/mars_quadrotor_msgs/package.xml`
- `mars_uav_sim/mars_quadrotor_msgs/ros/ros1.CMakeLists.txt`
- `mars_uav_sim/mars_quadrotor_msgs/ros/ros1.package.xml`
- `scripts/preflight_nationals_px4_sitl_env.sh`
- This document

The message package merge only added `Px4ctrlDebug.msg` plus its `std_msgs` dependency. `PositionCommand.msg` was not changed.

### D. Imported But Build-Gated

The following source modules are present but gated from automatic catkin discovery by `realflight_modules/CATKIN_IGNORE`:

- `realflight_modules/mid360_fastlio/FAST_LIO`
- `realflight_modules/mid360_fastlio/livox_ros_driver`
- `realflight_modules/mid360_fastlio/livox_ros_driver2`
- `realflight_modules/realsense-ros`

Reason: these modules can require hardware SDKs, sensor drivers, or system libraries that are not part of the PX4 SITL smoke path. They are kept in-tree for later realflight integration, but they must not break the main workspace build.

To enable a gated module later, add an explicit workspace symlink or remove/adjust the aggregate gate after installing its dependencies.

### E. Not Adopted

The teammate branch does not contain the current branch's nationals mock, Gazebo mock, boundary validation, PX4 SITL smoke, and preflight work. Those files were not overwritten.

The teammate branch also has older or narrower docs/scripts coverage for the current SITL validation path, so current branch docs/scripts remain authoritative.

## Workspace Links

These packages are intentionally built through explicit workspace links:

```text
~/super_ws/src/px4ctrl -> ~/super_ws/src/SUPER_DRONE/realflight_modules/px4ctrl
~/super_ws/src/uav_utils -> ~/super_ws/src/SUPER_DRONE/utils/uav_utils
~/super_ws/src/cmake_utils -> ~/super_ws/src/SUPER_DRONE/utils/cmake_utils
```

`scripts/preflight_nationals_px4_sitl_env.sh` checks and creates these links when the source directories exist.

## Module Status

- `px4ctrl`: integrated and built.
- `uav_utils`: integrated and built as a ROS package.
- `cmake_utils`: integrated and visible as a ROS package.
- FAST-LIO: source imported, build-gated by ancestor `realflight_modules/CATKIN_IGNORE`.
- Livox ROS driver: source imported, build-gated by ancestor `realflight_modules/CATKIN_IGNORE`.
- Livox ROS driver 2: source imported, build-gated by ancestor `realflight_modules/CATKIN_IGNORE`.
- Realsense ROS: source imported, build-gated by ancestor `realflight_modules/CATKIN_IGNORE`.

## Message Compatibility

`quadrotor_msgs/PositionCommand.msg` is unchanged.

Current MD5:

```text
d008e86de36e11deb1e4033ac2c394a9
```

Only `quadrotor_msgs/Px4ctrlDebug.msg` was added for px4ctrl debug publishing. No second `quadrotor_msgs` package was introduced.

## PX4 SITL Remaining Environment Gaps

PX4 SITL still requires local environment setup outside this repository:

- `PX4-Autopilot` must be installed or supplied with `PX4_DIR=/path/to/PX4-Autopilot`.
- GeographicLib `egm96-5.pgm` must be installed for MAVROS.

The preflight script reports both conditions clearly.

## Validation Results

Recorded during integration:

- `catkin_make -DCMAKE_BUILD_TYPE=Release`: PASS.
- `rospack find px4ctrl`: PASS.
- `rospack find uav_utils`: PASS.
- `rospack find cmake_utils`: PASS.
- `rosmsg md5 quadrotor_msgs/PositionCommand`: `d008e86de36e11deb1e4033ac2c394a9`.
- `./scripts/run_obstacle_course_validation.sh`: PASS.
- `./scripts/run_nationals_super_validation.sh`: PASS.
  - `in_bounds`: OK.
  - `goal sequence`: OK, `reached=5/5`.
  - forbidden flight-control publishers: OK, `px4ctrl_takeoff_land=absent`, `mavros_attitude=absent`.
- `./scripts/preflight_nationals_px4_sitl_env.sh`: clear FAIL because `PX4_DIR=missing`.
  - GeographicLib: OK.
  - `nationals_sim`: OK.
  - `px4ctrl`: OK.
  - `uav_utils`: OK.
  - `cmake_utils`: OK.
  - `catkin_make`: OK.

This integration did not start PX4 SITL, MAVROS, px4ctrl, real flight controllers, arm, takeoff, or land.
