#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

class SimValidator {
public:
    explicit SimValidator(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("validation_timeout", validation_timeout_, 90.0);
        nh_.param("reach_radius", default_reach_radius_, 0.6);
        loadGoals();

        cloud_sub_ = nh_.subscribe("/cloud_registered", 20, &SimValidator::cloudCallback, this);
        odom_sub_ = nh_.subscribe("/Odom_high_freq", 100, &SimValidator::odomCallback, this);
        goal_sub_ = nh_.subscribe("/planning/click_goal", 10, &SimValidator::goalCallback, this);
        cmd_sub_ = nh_.subscribe("/position_cmd", 100, &SimValidator::cmdCallback, this);
        status_sub_ = nh_.subscribe("/mock_drone/status", 20, &SimValidator::statusCallback, this);
        status_pub_ = nh_.advertise<std_msgs::String>("/competition_sim/validation_status", 1, true);
        timer_ = nh_.createTimer(ros::Duration(0.2), &SimValidator::timerCallback, this);
        start_time_ = ros::Time::now();
    }

private:
    struct Goal {
        double x;
        double y;
        double z;
        double reach_radius;
    };

    void loadGoals() {
        goals_.push_back(loadGoal("goal1", 1.5, 0.0, 1.0));
        goals_.push_back(loadGoal("goal2", 3.0, 0.0, 1.0));
        goals_.push_back(loadGoal("goal3", 5.0, 0.0, 1.0));
        goals_.push_back(loadGoal("goal4", 7.2, 0.0, 1.0));
        goals_.push_back(loadGoal("final", 9.8, 0.0, 1.0));
    }

    Goal loadGoal(const std::string &prefix, const double x, const double y, const double z) {
        Goal goal{x, y, z, default_reach_radius_};
        nh_.param(prefix + "_x", goal.x, x);
        nh_.param(prefix + "_y", goal.y, y);
        nh_.param(prefix + "_z", goal.z, z);
        nh_.param(prefix + "_reach_radius", goal.reach_radius, default_reach_radius_);
        return goal;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &) {
        countMessage(cloud_count_, first_cloud_time_);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
        countMessage(odom_count_, first_odom_time_);
        have_odom_ = true;
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;
        updateGoalProgress();
    }

    void goalCallback(const geometry_msgs::PoseStampedConstPtr &) {
        ++goal_msg_count_;
    }

    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
        countMessage(cmd_count_, first_cmd_time_);
        const bool finite = finitePoint(msg->position) &&
                            finiteVector(msg->velocity) &&
                            finiteVector(msg->acceleration) &&
                            std::isfinite(msg->yaw);
        if (!finite) {
            cmd_finite_ = false;
        }
    }

    void statusCallback(const std_msgs::StringConstPtr &) {
        ++status_count_;
    }

    void countMessage(int &count, ros::Time &first_time) {
        if (count == 0) {
            first_time = ros::Time::now();
        }
        ++count;
    }

    bool finitePoint(const geometry_msgs::Point &p) const {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    bool finiteVector(const geometry_msgs::Vector3 &v) const {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    void updateGoalProgress() {
        if (next_goal_idx_ >= goals_.size()) {
            return;
        }
        const Goal &goal = goals_[next_goal_idx_];
        const double dx = odom_x_ - goal.x;
        const double dy = odom_y_ - goal.y;
        const double dz = odom_z_ - goal.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= goal.reach_radius) {
            ++next_goal_idx_;
            ROS_INFO_STREAM("[sim_validator] reached expected goal "
                            << next_goal_idx_ << "/" << goals_.size());
        }
    }

    void timerCallback(const ros::TimerEvent &) {
        if (finished_) {
            return;
        }

        if (hasForbiddenPublisher("/px4ctrl/takeoff_land")) {
            forbidden_takeoff_land_pub_ = true;
        }
        if (hasForbiddenPublisher("/mavros/setpoint_raw/attitude")) {
            forbidden_mavros_attitude_pub_ = true;
        }

        if (next_goal_idx_ >= goals_.size()) {
            finish(true);
            return;
        }

        if ((ros::Time::now() - start_time_).toSec() >= validation_timeout_) {
            finish(false);
        }
    }

    bool hasForbiddenPublisher(const std::string &topic) const {
        XmlRpc::XmlRpcValue request;
        XmlRpc::XmlRpcValue response;
        XmlRpc::XmlRpcValue payload;
        request.setSize(1);
        request[0] = ros::this_node::getName();
        if (!ros::master::execute("getSystemState", request, response, payload, true)) {
            return false;
        }
        if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1) {
            return false;
        }
        XmlRpc::XmlRpcValue &publishers = payload[0];
        if (publishers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            return false;
        }
        for (int i = 0; i < publishers.size(); ++i) {
            XmlRpc::XmlRpcValue &entry = publishers[i];
            if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) {
                continue;
            }
            const std::string published_topic = static_cast<std::string>(entry[0]);
            XmlRpc::XmlRpcValue &nodes = entry[1];
            if (published_topic == topic &&
                nodes.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                nodes.size() > 0) {
                return true;
            }
        }
        return false;
    }

    double rate(const int count, const ros::Time &first_time) const {
        if (count < 2 || first_time.isZero()) {
            return 0.0;
        }
        const double elapsed = (ros::Time::now() - first_time).toSec();
        if (elapsed <= 0.0) {
            return 0.0;
        }
        return static_cast<double>(count - 1) / elapsed;
    }

    void finish(const bool goals_complete) {
        finished_ = true;

        const double cmd_rate = rate(cmd_count_, first_cmd_time_);
        const double odom_rate = rate(odom_count_, first_odom_time_);
        const double cloud_rate = rate(cloud_count_, first_cloud_time_);

        const bool cloud_ok = cloud_count_ > 0 && cloud_rate > 2.0;
        const bool odom_ok = odom_count_ > 0 && odom_rate > 50.0;
        const bool goal_ok = goal_msg_count_ > 0;
        const bool cmd_ok = cmd_count_ > 0 && cmd_rate > 20.0;
        const bool status_ok = status_count_ > 0;
        const bool forbidden_ok = !forbidden_takeoff_land_pub_ && !forbidden_mavros_attitude_pub_;
        const bool pass = goals_complete && cloud_ok && odom_ok && goal_ok &&
                          cmd_ok && status_ok && cmd_finite_ && forbidden_ok;

        std::ostringstream report;
        report << (pass ? "[PASS]" : "[FAIL]") << " competition sim validation\n"
               << "  cloud_registered: " << result(cloud_ok) << " count=" << cloud_count_
               << " rate=" << cloud_rate << "Hz\n"
               << "  Odom_high_freq: " << result(odom_ok) << " count=" << odom_count_
               << " rate=" << odom_rate << "Hz\n"
               << "  planning/click_goal: " << result(goal_ok) << " count=" << goal_msg_count_ << "\n"
               << "  position_cmd: " << result(cmd_ok) << " count=" << cmd_count_
               << " rate=" << cmd_rate << "Hz\n"
               << "  mock_drone/status: " << result(status_ok) << " count=" << status_count_ << "\n"
               << "  position_cmd finite: " << result(cmd_finite_) << "\n"
               << "  forbidden flight-control publishers: " << result(forbidden_ok)
               << " px4ctrl_takeoff_land=" << (forbidden_takeoff_land_pub_ ? "present" : "absent")
               << " mavros_attitude=" << (forbidden_mavros_attitude_pub_ ? "present" : "absent") << "\n"
               << "  goal sequence: " << result(goals_complete) << " reached="
               << next_goal_idx_ << "/" << goals_.size() << "\n";

        std_msgs::String status;
        status.data = report.str();
        status_pub_.publish(status);
        std::cout << report.str() << std::flush;
        ros::shutdown();
    }

    std::string result(const bool ok) const {
        return ok ? "OK" : "FAIL";
    }

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber status_sub_;
    ros::Publisher status_pub_;
    ros::Timer timer_;

    std::vector<Goal> goals_;
    double validation_timeout_{90.0};
    double default_reach_radius_{0.6};
    ros::Time start_time_;
    ros::Time first_cloud_time_;
    ros::Time first_odom_time_;
    ros::Time first_cmd_time_;

    int cloud_count_{0};
    int odom_count_{0};
    int cmd_count_{0};
    int goal_msg_count_{0};
    int status_count_{0};
    bool have_odom_{false};
    bool cmd_finite_{true};
    bool forbidden_takeoff_land_pub_{false};
    bool forbidden_mavros_attitude_pub_{false};
    bool finished_{false};
    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_z_{0.0};
    size_t next_goal_idx_{0};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_validator");
    ros::NodeHandle nh("~");
    SimValidator node(nh);
    ros::spin();
    return 0;
}
