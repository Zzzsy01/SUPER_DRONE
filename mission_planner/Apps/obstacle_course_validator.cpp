#include <cmath>
#include <iostream>
#include <sstream>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

class ObstacleCourseValidator {
public:
    explicit ObstacleCourseValidator(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("goal_x", goal_x_, 8.0);
        nh_.param("goal_y", goal_y_, 0.0);
        nh_.param("goal_z", goal_z_, 1.0);
        nh_.param("goal_radius", goal_radius_, 0.5);
        nh_.param("validation_timeout", validation_timeout_, 120.0);
        nh_.param("min_position_cmd_rate", min_cmd_rate_, 20.0);
        nh_.param("required_lateral_deviation", required_lateral_deviation_, 1.2);
        nh_.param("obstacle_min_x", obstacle_min_x_, 3.5);
        nh_.param("obstacle_max_x", obstacle_max_x_, 4.5);
        nh_.param("obstacle_min_y", obstacle_min_y_, -1.0);
        nh_.param("obstacle_max_y", obstacle_max_y_, 1.0);
        nh_.param("obstacle_min_z", obstacle_min_z_, 0.0);
        nh_.param("obstacle_max_z", obstacle_max_z_, 2.0);
        nh_.param("safety_margin", safety_margin_, 0.05);

        odom_sub_ = nh_.subscribe("/Odom_high_freq", 100, &ObstacleCourseValidator::odomCallback, this);
        cmd_sub_ = nh_.subscribe("/position_cmd", 100, &ObstacleCourseValidator::cmdCallback, this);
        cloud_sub_ = nh_.subscribe("/cloud_registered", 20, &ObstacleCourseValidator::cloudCallback, this);
        goal_sub_ = nh_.subscribe("/planning/click_goal", 10, &ObstacleCourseValidator::goalCallback, this);
        status_pub_ = nh_.advertise<std_msgs::String>("/obstacle_course/validation_status", 1, true);
        timer_ = nh_.createTimer(ros::Duration(0.2), &ObstacleCourseValidator::timerCallback, this);
        start_time_ = ros::Time::now();
    }

private:
    void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
        have_odom_ = true;
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double z = msg->pose.pose.position.z;
        max_abs_y_ = std::max(max_abs_y_, std::fabs(y));

        const bool in_obstacle = x >= obstacle_min_x_ - safety_margin_ &&
                                 x <= obstacle_max_x_ + safety_margin_ &&
                                 y >= obstacle_min_y_ - safety_margin_ &&
                                 y <= obstacle_max_y_ + safety_margin_ &&
                                 z >= obstacle_min_z_ - safety_margin_ &&
                                 z <= obstacle_max_z_ + safety_margin_;
        if (in_obstacle) {
            entered_obstacle_ = true;
        }

        const double dx = x - goal_x_;
        const double dy = y - goal_y_;
        const double dz = z - goal_z_;
        reached_goal_ = std::sqrt(dx * dx + dy * dy + dz * dz) <= goal_radius_;
    }

    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
        if (cmd_count_ == 0) {
            first_cmd_time_ = ros::Time::now();
        }
        ++cmd_count_;
        if (!finitePoint(msg->position) ||
            !finiteVector(msg->velocity) ||
            !finiteVector(msg->acceleration) ||
            !std::isfinite(msg->yaw)) {
            cmd_finite_ = false;
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
        ++cloud_count_;
        if (msg->width == 0 || msg->height == 0) {
            cloud_nonempty_ = false;
        }
    }

    void goalCallback(const geometry_msgs::PoseStampedConstPtr &) {
        ++goal_count_;
    }

    bool finitePoint(const geometry_msgs::Point &p) const {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    bool finiteVector(const geometry_msgs::Vector3 &v) const {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    double cmdRate() const {
        if (cmd_count_ < 2 || first_cmd_time_.isZero()) {
            return 0.0;
        }
        const double elapsed = (ros::Time::now() - first_cmd_time_).toSec();
        return elapsed > 0.0 ? static_cast<double>(cmd_count_ - 1) / elapsed : 0.0;
    }

    void timerCallback(const ros::TimerEvent &) {
        if (finished_) {
            return;
        }

        if (reached_goal_) {
            finish();
            return;
        }

        if ((ros::Time::now() - start_time_).toSec() >= validation_timeout_) {
            finish();
        }
    }

    void finish() {
        finished_ = true;
        const double rate = cmdRate();
        const bool cmd_rate_ok = rate >= min_cmd_rate_;
        const bool cloud_ok = cloud_count_ > 0 && cloud_nonempty_;
        const bool goal_ok = goal_count_ > 0;
        const bool avoided_obstacle = !entered_obstacle_;
        const bool deviated = max_abs_y_ > required_lateral_deviation_;
        const bool pass = have_odom_ && cloud_ok && goal_ok && reached_goal_ &&
                          cmd_rate_ok && cmd_finite_ && avoided_obstacle && deviated;

        std::ostringstream report;
        report << (pass ? "[PASS]" : "[FAIL]") << " obstacle course validation\n"
               << "  odom: " << result(have_odom_) << "\n"
               << "  cloud_registered: " << result(cloud_ok) << " count=" << cloud_count_ << "\n"
               << "  planning/click_goal: " << result(goal_ok) << " count=" << goal_count_ << "\n"
               << "  position_cmd_rate: " << result(cmd_rate_ok) << " rate=" << rate << "Hz\n"
               << "  position_cmd_finite: " << result(cmd_finite_) << "\n"
               << "  reached_goal: " << result(reached_goal_) << "\n"
               << "  did_not_enter_obstacle: " << result(avoided_obstacle) << "\n"
               << "  lateral_deviation: " << result(deviated) << " max_abs_y=" << max_abs_y_ << "\n";
        if (!pass) {
            report << "  reason: " << failureReason(cmd_rate_ok, cloud_ok, goal_ok, avoided_obstacle, deviated) << "\n";
        }

        std_msgs::String status;
        status.data = report.str();
        status_pub_.publish(status);
        std::cout << report.str() << std::flush;
        ros::shutdown();
    }

    std::string result(const bool ok) const {
        return ok ? "OK" : "FAIL";
    }

    std::string failureReason(const bool cmd_rate_ok, const bool cloud_ok, const bool goal_ok,
                              const bool avoided_obstacle, const bool deviated) const {
        if (!cloud_ok) return "missing or empty /cloud_registered";
        if (!goal_ok) return "no /planning/click_goal received";
        if (!have_odom_) return "no /Odom_high_freq received";
        if (!cmd_rate_ok) return "/position_cmd rate below threshold";
        if (!cmd_finite_) return "/position_cmd contains nan or inf";
        if (!reached_goal_) return "goal not reached before timeout";
        if (!avoided_obstacle) return "odom entered configured obstacle volume";
        if (!deviated) return "trajectory did not deviate enough from the blocked straight line";
        return "unknown";
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher status_pub_;
    ros::Timer timer_;
    ros::Time start_time_;
    ros::Time first_cmd_time_;

    double goal_x_{8.0};
    double goal_y_{0.0};
    double goal_z_{1.0};
    double goal_radius_{0.5};
    double validation_timeout_{120.0};
    double min_cmd_rate_{20.0};
    double required_lateral_deviation_{1.2};
    double obstacle_min_x_{3.5};
    double obstacle_max_x_{4.5};
    double obstacle_min_y_{-1.0};
    double obstacle_max_y_{1.0};
    double obstacle_min_z_{0.0};
    double obstacle_max_z_{2.0};
    double safety_margin_{0.05};
    double max_abs_y_{0.0};
    int cmd_count_{0};
    int cloud_count_{0};
    int goal_count_{0};
    bool have_odom_{false};
    bool cmd_finite_{true};
    bool cloud_nonempty_{true};
    bool reached_goal_{false};
    bool entered_obstacle_{false};
    bool finished_{false};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_course_validator");
    ros::NodeHandle nh("~");
    ObstacleCourseValidator node(nh);
    ros::spin();
    return 0;
}
