#include "ros/ros.h"

#define BACKWARD_HAS_DW 1
#include "utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

#include "waypoint_mission/super_drone_mission.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "super_drone_mission");
    ros::NodeHandle nh("~");

    mission_planner::SuperDroneMission mission(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}
