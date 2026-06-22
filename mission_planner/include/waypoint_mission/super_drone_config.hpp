#ifndef MISSION_PLANNER_SUPER_DRONE_CONFIG_HPP
#define MISSION_PLANNER_SUPER_DRONE_CONFIG_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <utils/color_text.hpp>
#include <utils/eigen_alias.hpp>
#include <utils/yaml_loader.hpp>

namespace mission_planner {
    using namespace super_utils;
    using namespace color_text;
    using namespace std;

    enum SuperDroneTriggerType {
        SUPER_DRONE_DELAY = 2,
        SUPER_DRONE_PX4CTRL = 3
    };

    class SuperDroneMissionConfig {
    public:
        int start_trigger_type{SUPER_DRONE_PX4CTRL};
        double start_program_delay{3.0};
        double landing_trigger_delay{1.5};
        double odom_timeout{0.2};
        double publish_dt{0.2};
        string goal_pub_topic{"/planning/click_goal"};
        string odom_topic{"/Odom_high_freq"};
        string start_trigger_topic{"/traj_start_trigger"};
        string land_topic{"/px4ctrl/takeoff_land"};
        vec_E<Vec3f> waypoints;
        vector<double> switch_dis_vec;

        static double str2double(const string &s) {
            double d = 0.0;
            stringstream ss;
            ss << s;
            ss >> setprecision(16) >> d;
            return d;
        }

        void LoadWaypoint(const string &file_name) {
            ifstream the_file(file_name);
            string line;
            Vec3f waypoint;
            while (getline(the_file, line)) {
                if (line.empty()) {
                    continue;
                }
                vector<string> result;
                istringstream iss(line);
                for (string s; iss >> s;) {
                    result.push_back(s);
                }
                if (result.size() < 4) {
                    continue;
                }
                for (size_t i = 0; i < 3; i++) {
                    waypoint(i) = str2double(result[i]);
                }
                switch_dis_vec.push_back(str2double(result[3]));
                waypoints.push_back(waypoint);
            }

            cout << GREEN << " -- [SUPER_DRONE] Load " << waypoints.size() << " waypoints." << RESET << endl;
            for (size_t i = 0; i < waypoints.size(); i++) {
                cout << BLUE << " -- [SUPER_DRONE] Waypoint " << i
                     << " at (" << waypoints[i].x() << ", " << waypoints[i].y() << ", " << waypoints[i].z()
                     << "), switch_dis = " << switch_dis_vec[i] << RESET << endl;
            }
        }

        SuperDroneMissionConfig() = default;

        explicit SuperDroneMissionConfig(const string &cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);

            loader.LoadParam("start_trigger_type", start_trigger_type, static_cast<int>(SUPER_DRONE_PX4CTRL));
            loader.LoadParam("start_program_delay", start_program_delay, 3.0);
            loader.LoadParam("landing_trigger_delay", landing_trigger_delay, 1.5);
            loader.LoadParam("odom_timeout", odom_timeout, 0.2);
            loader.LoadParam("publish_dt", publish_dt, 0.2);
            loader.LoadParam("goal_pub_topic", goal_pub_topic, string("/planning/click_goal"));
            loader.LoadParam("odom_topic", odom_topic, string("/Odom_high_freq"));
            loader.LoadParam("start_trigger_topic", start_trigger_topic, string("/traj_start_trigger"));
            loader.LoadParam("land_topic", land_topic, string("/px4ctrl/takeoff_land"));
        }
    };
}

#endif
