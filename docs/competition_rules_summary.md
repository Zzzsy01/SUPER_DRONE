# Competition Rules Summary

Source files read:

- `docs/reference/具身智能赛道-空中具身智能挑战赛规则0508.pdf`
- `docs/reference/空中具身智能赛项规则文件 (1).pdf`

Priority: if the two versions disagree, use the `0508` adjusted version.

## Field

- Official field size in the `0508` adjusted rules: `12 m x 8 m x 2 m`.
- Older rule version states `8 m x 8 m x 2 m`; this is superseded by the `0508` adjusted version.
- The field has a takeoff zone and landing zone.
- The takeoff zone is at coordinate origin `[0, 0]`.
- The landing zone center will be provided before the competition.

## Takeoff Zone

- Takeoff begins from the takeoff zone near `[0, 0]`.
- The robot must autonomously take off for the formal task.
- Older version text described takeoff/landing as `0.5 m x 0.5 m` square regions; the `0508` extracted text does not repeat that square-size detail clearly, so confirm the physical takeoff-zone marking onsite.

## Four Areas and Four Scoring Rings

The field is divided into four numbered task areas.

- Area 1: structured obstacle area.
- Area 2: dense forest area.
- Area 3: moving obstacle area.
- Area 4: arched obstacle area.

After each area there is one scoring ring. The robot must pass the four areas and then the four scoring rings in sequence.

- Scoring ring radius: `0.5 m`.
- Scoring ring center positions will be provided before the competition.
- Each scoring ring is worth `18` points.

## Landing Zone

- After passing all four scoring rings, the robot must reach the landing zone and autonomously land.
- In the `0508` rules, precision landing is scored by horizontal distance from UAV center to landing-zone center:
  - `<= 15 cm`: full landing score.
  - `15 cm` to `25 cm`: `8` points.
  - `> 25 cm`: `5` points.
  - loss of control or manual takeover landing: `0` points.
- Older rule text used motor-projection-in-KT-board wording; use the `0508` distance-based rule unless the organizer states otherwise.

## Obstacles

Area 1:

- `8-10` rectangular cuboid obstacles.
- Each cuboid size: `0.5 m x 0.5 m x 2 m`.

Area 2:

- `5-7` artificial trees.
- Tree height: not less than `1.8 m`.
- The `0508` version also mentions a fan in a corner for wind disturbance.

Area 3:

- `2` moving obstacles.
- Each moving obstacle size: `0.5 m x 0.5 m x 2 m`.
- Moving speed: not greater than `0.5 m/s`.

Area 4:

- Arched obstacle area.
- Two passable arches.
- Each arch passable width: `0.7 m`.

## Height Limits

- Formal onsite field height is `2 m`.
- Online provincial self-built selection field has a flight height limit of `1.5 m`.
- The PDF text does not clearly state a separate formal-round maximum flight altitude beyond the field height; confirm onsite if a stricter altitude limit is announced.

## Autonomous Requirements

- After the competition starts, the robot must move autonomously.
- Remote control is not allowed after takeoff unless a loss-of-control emergency occurs.
- The robot must autonomously take off, fly through the task areas and scoring rings, and autonomously land.
- The robot must have a one-key motor stop function.

## Scoring Overview

- Autonomous takeoff: `10` points.
- Four scoring rings: `18` points each, `72` total.
- Autonomous landing: `10` points.
- Collision score: `8` points. Each collision with obstacles or field net deducts `2` points. If full flight cannot be completed, this item is `0`.

## Online / Offline Submission and Rosbag

Provincial online selection:

- Teams build a self-defined structured-obstacle field.
- Self-built field size: not less than `5 m x 5 m`.
- Place `9` rectangular obstacles, each `0.5 m x 0.5 m x 1.5 m`, following the required layout.
- Start and end positions must match the required diagram.
- Flight height must not exceed `1.5 m`; exceeding height causes penalties.
- Submit proof materials including:
  - Photos of the built environment.
  - Robot photos and specifications.
  - One-key motor stop demonstration video.
  - Third-person full flight video.
  - Screen recording from program start to task completion; RViz should show robot position, real-time mapping visualization, flight trajectory, and point cloud or map information.
  - Rosbag data package.
  - Technical report covering localization, perception/mapping, trajectory planning, and control.

Rosbag requirement:

- The extracted PDF text clearly requires a rosbag data package and says it should include odometry and real-time mapping/perception-related topics.
- The exact full topic list is partially line-wrapped in PDF extraction and should be manually verified from the source PDF before final submission.

## Fields Needing Manual Confirmation

- Exact takeoff-zone and landing-zone physical marking dimensions for the `0508` version.
- Final scoring ring center coordinates.
- Final landing-zone center coordinate.
- Full required rosbag topic list from the PDF, because text extraction split several topic lines.
- Any onsite altitude restriction stricter than the `2 m` field height.
