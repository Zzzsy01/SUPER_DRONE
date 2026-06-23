cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DUSE_ROS1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        sensor_msgs
        geometry_msgs
        gazebo_msgs
        nav_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
        scripts/nationals_layout_cloud_publisher.py
        scripts/generate_nationals_waypoints.py
        scripts/nationals_sim_validator.py
        scripts/gazebo_mock_drone_pose_bridge.py
        scripts/nationals_waypoint_marker.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
        )

add_executable(waypoint_mission
        Apps/ros1_waypoint_mission.cpp
        )
target_link_libraries(waypoint_mission
        ${THIRD_PARTY}
        )

add_executable(super_drone_mission
        Apps/ros1_super_drone_mission.cpp
        )
target_link_libraries(super_drone_mission
        ${THIRD_PARTY}
        )

add_executable(goal_feeder
        Apps/ros1_goal_feeder.cpp
        )
target_link_libraries(goal_feeder
        ${THIRD_PARTY}
        )

add_executable(competition_map_publisher
        Apps/competition_map_publisher.cpp
        )
target_link_libraries(competition_map_publisher
        ${THIRD_PARTY}
        )

add_executable(mock_drone_tracker
        Apps/mock_drone_tracker.cpp
        )
target_link_libraries(mock_drone_tracker
        ${THIRD_PARTY}
        )

add_executable(competition_goal_sequence
        Apps/competition_goal_sequence.cpp
        )
target_link_libraries(competition_goal_sequence
        ${THIRD_PARTY}
        )

add_executable(sim_validator
        Apps/sim_validator.cpp
        )
target_link_libraries(sim_validator
        ${THIRD_PARTY}
        )

add_executable(gazebo_cloud_bridge
        Apps/gazebo_cloud_bridge.cpp
        )
target_link_libraries(gazebo_cloud_bridge
        ${THIRD_PARTY}
        )

add_executable(gazebo_mock_drone_tracker
        Apps/gazebo_mock_drone_tracker.cpp
        )
target_link_libraries(gazebo_mock_drone_tracker
        ${THIRD_PARTY}
        )

add_executable(obstacle_course_validator
        Apps/obstacle_course_validator.cpp
        )
target_link_libraries(obstacle_course_validator
        ${THIRD_PARTY}
        )
