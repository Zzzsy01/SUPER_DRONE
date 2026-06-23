# Nationals PX4 SITL Integration Runbook

This is a future-stage runbook. Do not run it automatically from validation
scripts, and do not use it for a real vehicle.

## Required Boundaries

Use `start_mission_driver:=false` while bringing up PX4 SITL. Start the mission
driver only after odometry, point cloud input, MAVROS state, and controller mode
are checked.

Use `px4ctrl no_RC:=true` or a SITL-specific px4ctrl configuration. Do not use a
real-flight config without reviewing arming, takeoff, failsafe, and RC behavior.

Relay `/mavros/local_position/odom` to `/Odom_high_freq` so SUPER receives the
same odometry topic used by the validated simulation chain.

Make sure `/cloud_registered` has a source. Real Gazebo lidar is preferred when
available, but `nationals_layout_cloud_publisher.py` is acceptable as a temporary
planning-side point cloud source while validating the software integration.

## Checks Before Mission Start

1. PX4 SITL is running and MAVROS is connected.
2. `/mavros/local_position/odom` is publishing.
3. `/Odom_high_freq` relay is publishing at the expected rate.
4. `/cloud_registered` is publishing and non-empty.
5. SUPER publishes finite `/position_cmd`.
6. px4ctrl accepts SITL setpoints in the intended mode.
7. `start_mission_driver:=false` remains set until manual confirmation.

## Safety

Do not install propellers. Do not use this runbook for a real aircraft. This
document is only for PX4 SITL integration after the mock validation has passed.
