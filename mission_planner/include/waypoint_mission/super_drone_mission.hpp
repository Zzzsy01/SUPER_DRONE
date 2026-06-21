#ifndef MISSION_PLANNER_SUPER_DRONE_MISSION_HPP
#define MISSION_PLANNER_SUPER_DRONE_MISSION_HPP

#include <string>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>

#include "super_drone_config.hpp"

namespace mission_planner {
    using namespace std;
    using namespace super_utils;

    class SuperDroneMission {
    private:
        SuperDroneMissionConfig cfg_;
        ros::NodeHandle nh_;

        Eigen::Vector3d cur_position{Eigen::Vector3d::Zero()};
        int waypoint_counter{0};
        bool had_odom{false};
        bool triggered{false};
        bool landing_stage{false};
        bool land_sent{false};
        bool new_goal{true};
        bool trigger_once{false};
        double odom_rcv_time{0.0};
        double system_start_time{0.0};
        double last_pub_time_{0.0};
        double landing_stage_start_time_{0.0};

        ros::Publisher goal_pub_;
        ros::Publisher land_pub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber start_trigger_sub_;
        ros::Timer mission_timer_;

        void resetMission() {
            triggered = true;
            landing_stage = false;
            land_sent = false;
            new_goal = true;
            trigger_once = true;
            waypoint_counter = 0;
            last_pub_time_ = 0.0;
            landing_stage_start_time_ = 0.0;
        }

        void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_odom = true;
            odom_rcv_time = ros::Time::now().toSec();
            cur_position = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
        }

        void StartTriggerCallback(const geometry_msgs::PoseStampedConstPtr &) {
            if (cfg_.waypoints.empty()) {
                ROS_WARN("[SUPER_DRONE] No waypoints loaded, skip trigger.");
                return;
            }
            resetMission();
            cout << YELLOW << " -- [SUPER_DRONE] Trigger received, mission start." << RESET << endl;
        }

        bool closeToCurrentWaypoint() const {
            if (cfg_.waypoints.empty()) {
                return false;
            }
            const Vec3f &goal = cfg_.waypoints[waypoint_counter];
            return (goal - cur_position).norm() < cfg_.switch_dis_vec[waypoint_counter];
        }

        void publishCurrentGoal(const double cur_t) {
            geometry_msgs::PoseStamped goal;
            goal.pose.position.x = cfg_.waypoints[waypoint_counter].x();
            goal.pose.position.y = cfg_.waypoints[waypoint_counter].y();
            goal.pose.position.z = cfg_.waypoints[waypoint_counter].z();
            goal.pose.orientation.w = 1.0;
            goal.header.frame_id = "world";
            goal.header.stamp = ros::Time::now();
            goal_pub_.publish(goal);
            last_pub_time_ = cur_t;
            new_goal = false;
            cout << YELLOW << " -- [SUPER_DRONE] Publish goal "
                 << waypoint_counter << " -> " << cfg_.waypoints[waypoint_counter].transpose()
                 << RESET << endl;
        }

        void publishLandCommand() {
            quadrotor_msgs::TakeoffLand msg;
            msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
            land_pub_.publish(msg);
            land_sent = true;
            triggered = false;
            cout << RED << " -- [SUPER_DRONE] Mission finished, send LAND." << RESET << endl;
        }

        void MissionTimerCallback(const ros::TimerEvent &) {
            const double cur_t = ros::Time::now().toSec();

            if (cfg_.start_trigger_type == SUPER_DRONE_DELAY && !trigger_once) {
                if (cur_t - system_start_time >= cfg_.start_program_delay) {
                    resetMission();
                    cout << YELLOW << " -- [SUPER_DRONE] Delay trigger fired." << RESET << endl;
                } else {
                    return;
                }
            }

            if (!triggered && !landing_stage) {
                return;
            }

            if (!had_odom || cur_t - odom_rcv_time > cfg_.odom_timeout) {
                static double last_print_t = 0.0;
                if (cur_t - last_print_t > 1.0) {
                    last_print_t = cur_t;
                    cout << YELLOW << " -- [SUPER_DRONE] Odom timeout." << RESET << endl;
                }
                return;
            }

            if (landing_stage) {
                if (new_goal || cur_t - last_pub_time_ > cfg_.publish_dt) {
                    publishCurrentGoal(cur_t);
                }
                if (!land_sent && cur_t - landing_stage_start_time_ >= cfg_.landing_trigger_delay) {
                    publishLandCommand();
                }
                return;
            }

            if (closeToCurrentWaypoint()) {
                if (waypoint_counter + 1 < static_cast<int>(cfg_.waypoints.size())) {
                    waypoint_counter++;
                    new_goal = true;
                } else {
                    landing_stage = true;
                    new_goal = true;
                    landing_stage_start_time_ = cur_t;
                    cout << GREEN << " -- [SUPER_DRONE] Reach final hover point, prepare landing." << RESET << endl;
                }
            }

            if (new_goal || cur_t - last_pub_time_ > cfg_.publish_dt) {
                publishCurrentGoal(cur_t);
            }
        }

    public:
        SuperDroneMission() = default;

        explicit SuperDroneMission(const ros::NodeHandle &nh) {
            nh_ = nh;
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/" + name))
            std::string dft_cfg_path = CONFIG_FILE_DIR("super_drone_waypoint.yaml");
            std::string cfg_path, cfg_name;
            if (nh.param("config_path", cfg_path, dft_cfg_path)) {
                cout << " -- [SUPER_DRONE] Load config from: " << cfg_path << endl;
            } else if (nh.param("config_name", cfg_name, dft_cfg_path)) {
                cfg_path = CONFIG_FILE_DIR(cfg_name);
                cout << " -- [SUPER_DRONE] Load config by file name: " << cfg_name << endl;
            }
#define DATA_FILE_DIR(name) (string(string(ROOT_DIR) + "data/" + name))
            std::string dft_data_path = DATA_FILE_DIR("super_drone_competition_template.txt");
            std::string data_path, data_name;
            if (nh.param("data_path", data_path, dft_data_path)) {
                cout << " -- [SUPER_DRONE] Load mission data from: " << data_path << endl;
            } else if (nh.param("data_name", data_name, dft_data_path)) {
                data_path = DATA_FILE_DIR(data_name);
                cout << " -- [SUPER_DRONE] Load mission data by file name: " << data_name << endl;
            }

            cfg_ = SuperDroneMissionConfig(cfg_path);
            cfg_.LoadWaypoint(data_path);

            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &SuperDroneMission::OdomCallback, this);
            if (cfg_.start_trigger_type == SUPER_DRONE_PX4CTRL) {
                start_trigger_sub_ = nh_.subscribe(cfg_.start_trigger_topic, 1,
                                                   &SuperDroneMission::StartTriggerCallback, this);
            }

            mission_timer_ = nh_.createTimer(ros::Duration(0.01), &SuperDroneMission::MissionTimerCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(cfg_.land_topic, 1);
            system_start_time = ros::Time::now().toSec();
        }
    };
}

#endif
