# SUPER_DRONE

`SUPER_DRONE` is a ROS1 real-flight integration project based on [hku-mars/SUPER](https://github.com/hku-mars/SUPER), adapted for a `Livox MID-360` lidar platform without a depth camera.

This repository keeps `SUPER` as the main planner and adds a dedicated mission layer for competition flight. The integration idea references the waypoint-style mission flow used in `REAL_DRONE_400`, but all code changes are confined to the `SUPER` side. No `REAL_DRONE_400` source code is included or modified in this repository.

## 1. Repository Scope

Included in this repository:

- The `SUPER` source tree needed for planning, mapping, mission management, and ROS1 integration
- The new `SUPER_DRONE` mission interface, planner config, launch file, and message definition
- A rewritten root `README.md` that documents every modified and added file

Excluded from this repository:

- The original `.git` history from the upstream `SUPER` checkout
- ROS build outputs such as `build`, `devel`, `install`, and runtime `log` directories
- Large demo assets and simulation data that are not required for this real-flight integration, such as:
  - `misc/fig1.gif`
  - `misc/tracking.gif`
  - `misc/tailsitter.gif`
  - `misc/exp.gif`
  - `misc/scirobotics.ado6187.pdf`
  - `mars_uav_sim/perfect_drone_sim/pcd/random_map_150.pcd`
  - `mars_uav_sim/perfect_drone_sim/pcd/random_map_50.pcd`
  - `mars_uav_sim/perfect_drone_sim/pcd/random_map_2_26609.pcd`
  - `mars_uav_sim/perfect_drone_sim/pcd/random_map_24_6635.pcd`
  - `mars_uav_sim/perfect_drone_sim/meshes/yunque-M.dae`
  - `mars_uav_sim/perfect_drone_sim/meshes/R.jpeg`

## 2. What Was Changed in SUPER

The following existing files from `SUPER` were modified.

### 2.1 Message Registration

1. `mars_uav_sim/mars_quadrotor_msgs/CMakeLists.txt`
   - Added `TakeoffLand.msg` to `add_message_files(...)`
   - Purpose: allow the new mission node to publish a landing command

2. `mars_uav_sim/mars_quadrotor_msgs/ros/ros1.CMakeLists.txt`
   - Added `TakeoffLand.msg` to the ROS1 message registration list
   - Purpose: ensure ROS1 message generation includes the new landing message

### 2.2 Mission Planner Build and Dependency Wiring

3. `mission_planner/CMakeLists.txt`
   - Added a new executable:
     - `super_drone_mission`
     - source file: `Apps/ros1_super_drone_mission.cpp`
   - Purpose: build the dedicated competition mission node

4. `mission_planner/ros/ros1.CMakeLists.txt`
   - Added the same `super_drone_mission` executable for the ROS1 build path
   - Purpose: keep the ROS1 build flow complete and consistent

5. `mission_planner/package.xml`
   - Added:
     - `build_depend>quadrotor_msgs`
     - `build_export_depend>quadrotor_msgs`
     - `exec_depend>quadrotor_msgs`
   - Purpose: expose the new takeoff/landing message dependency to the package

6. `mission_planner/ros/ros1.package.xml`
   - Added the same `quadrotor_msgs` dependency entries
   - Purpose: keep the ROS1 package manifest aligned with the new mission node

## 3. New Files Added for SUPER_DRONE

The following files were added to create the `SUPER_DRONE` integration layer.

1. `mars_uav_sim/mars_quadrotor_msgs/msg/TakeoffLand.msg`
   - New ROS message:
     - `TAKEOFF = 1`
     - `LAND = 2`
     - `takeoff_land_cmd`
   - Purpose: provide a simple landing command interface for `/px4ctrl/takeoff_land`

2. `mission_planner/include/waypoint_mission/super_drone_config.hpp`
   - New config class: `SuperDroneMissionConfig`
   - Purpose:
     - load mission parameters from YAML
     - load waypoint text files
     - define topic names and timing parameters for the real-flight mission

3. `mission_planner/include/waypoint_mission/super_drone_mission.hpp`
   - New mission logic class: `SuperDroneMission`
   - Purpose:
     - subscribe to odometry
     - wait for takeoff trigger
     - publish waypoint goals to `SUPER`
     - switch to the next waypoint when within threshold distance
     - hold at the final point
     - send the final landing command

4. `mission_planner/Apps/ros1_super_drone_mission.cpp`
   - New ROS1 executable entry point
   - Purpose: launch the `SuperDroneMission` node

5. `mission_planner/config/super_drone_waypoint.yaml`
   - New mission-layer config file
   - Purpose: define mission topics and start/landing timing

6. `mission_planner/data/super_drone_competition_template.txt`
   - New waypoint template
   - Purpose: provide a placeholder competition route
   - Note: the last point is treated as the final hover point before landing

7. `mission_planner/launch/super_drone.launch`
   - New launch file
   - Purpose: start only:
     - `super_drone_mission`
     - `fsm_node`
   - This avoids pulling in simulator-only launch content

8. `super_planner/config/super_drone_ros1.yaml`
   - New planner-side config file for ROS1 real flight
   - Purpose:
     - connect the lidar point cloud topic
     - connect odometry
     - enable click-goal mode for mission handoff
     - publish planner output to the flight controller interface

## 4. SUPER_DRONE Overall Logic

The stitched system works like this:

1. `MID-360` point cloud is published to `/cloud_registered`
2. High-rate odometry is published to `/Odom_high_freq`
3. `super_planner` uses those two inputs to maintain the local map and generate a collision-free trajectory
4. `super_drone_mission` waits for `/traj_start_trigger`
5. After the trigger arrives, `super_drone_mission` sends one waypoint at a time to `/planning/click_goal`
6. `SUPER` replans toward that goal and publishes commands to `/position_cmd`
7. After the final hover point is reached, `super_drone_mission` waits for `landing_trigger_delay`
8. `super_drone_mission` publishes `quadrotor_msgs/TakeoffLand` with `LAND` to `/px4ctrl/takeoff_land`

## 5. Key Interfaces

### 5.1 Inputs to SUPER_DRONE

- `/cloud_registered`
  - Type: point cloud
  - Used by: `rog_map` through `super_planner/config/super_drone_ros1.yaml`

- `/Odom_high_freq`
  - Type: `nav_msgs/Odometry`
  - Used by:
    - `super_drone_mission`
    - `rog_map` and planner state in `SUPER`

- `/traj_start_trigger`
  - Type: `geometry_msgs/PoseStamped`
  - Used by: `super_drone_mission`
  - Meaning: start the waypoint mission after takeoff

### 5.2 Internal Handoff from Mission Layer to SUPER Planner

- `/planning/click_goal`
  - Type: `geometry_msgs/PoseStamped`
  - Published by: `super_drone_mission`
  - Consumed by: `fsm_node`
  - Meaning: each waypoint is injected into `SUPER` as a sequential target

### 5.3 Outputs from SUPER_DRONE

- `/position_cmd`
  - Type: planner position command
  - Published by: `SUPER`
  - Consumed by: the downstream controller

- `/px4ctrl/takeoff_land`
  - Type: `quadrotor_msgs/TakeoffLand`
  - Published by: `super_drone_mission`
  - Meaning: final landing command

## 6. Files You Will Most Likely Edit Before a Real Competition Flight

1. `mission_planner/data/super_drone_competition_template.txt`
   - Replace the placeholder waypoints with the actual route for your field

2. `mission_planner/config/super_drone_waypoint.yaml`
   - Change the mission topics or timing parameters if your PX4/trigger side differs

3. `super_planner/config/super_drone_ros1.yaml`
   - Change planner interfaces and dynamic limits
   - Especially:
     - `cmd_topic`
     - `cloud_topic`
     - `odom_topic`
     - `max_vel`
     - `max_acc`
     - map size and map resolution

## 7. Build

This repository is intended for a ROS1 catkin workspace.

Example:

```bash
cd <your_catkin_ws>/src
git clone https://github.com/kkkkkai-pro/SUPER_DRONE.git
cd ..
catkin_make
source devel/setup.bash
```

## 8. Launch

Example:

```bash
roslaunch mission_planner super_drone.launch
```

This launch file starts:

- `mission_planner/super_drone_mission`
- `super_planner/fsm_node`

## 9. Important Notes

- This repository is a cleaned integration snapshot, not the original upstream git history.
- The original upstream project is `hku-mars/SUPER`.
- The mission integration idea was designed for a ROS1 real-flight workflow with lidar-only perception.
- The current waypoint file is only a template and must be replaced before a real competition run.
- No build verification is bundled in this repository snapshot. Please run `catkin_make` in your own ROS1 environment and then check topics, frames, and controller interfaces on your platform.

## 10. Upstream Reference

- Upstream planner framework: [hku-mars/SUPER](https://github.com/hku-mars/SUPER)
- Trajectory-style competition workflow reference: [NEU-REAL/REAL_DRONE_400](https://github.com/NEU-REAL/REAL_DRONE_400)
