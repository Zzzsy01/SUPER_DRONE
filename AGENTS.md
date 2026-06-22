# AGENTS.md

## Project goal

This repository is for a ROS1 real-flight UAV competition system based on hku-mars/SUPER.

The intended architecture is:

MID-360 / FAST-LIO -> SUPER planner -> px4ctrl -> PX4

SUPER is the main planner. REAL_DRONE_400 is only used as a reference for the real-flight chain, PX4 control, and takeoff/landing workflow.

## Hard constraints

- Do not replace SUPER's core planner with EGO-Planner.
- Do not modify core trajectory optimization, Astar, CIRI, ROG-Map, or backup trajectory logic unless explicitly requested.
- Do not add a second quadrotor_msgs package.
- Do not add depth camera dependencies. The robot uses MID-360 lidar only.
- Keep ROS1 compatibility.
- Prefer small commits.
- After changing CMakeLists.txt, package.xml, msg files, or launch files, run catkin_make or clearly explain why it cannot be run.

## Key topics

Inputs:
- /cloud_registered
- /Odom_high_freq
- /traj_start_trigger

Planner handoff:
- /planning/click_goal

Outputs:
- /position_cmd
- /px4ctrl/takeoff_land

## Competition safety

Real-flight changes must be conservative. Do not increase velocity, acceleration, or planner aggressiveness without explicit approval.
