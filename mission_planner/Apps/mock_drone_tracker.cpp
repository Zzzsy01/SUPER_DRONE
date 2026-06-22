#include <algorithm>
#include <cmath>
#include <sstream>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Vec3 operator+(const Vec3 &a, const Vec3 &b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 &a, const Vec3 &b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 &a, const double s) {
    return {a.x * s, a.y * s, a.z * s};
}

double norm(const Vec3 &v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 limitNorm(const Vec3 &v, const double max_norm) {
    const double n = norm(v);
    if (n <= max_norm || n < 1.0e-9) {
        return v;
    }
    return v * (max_norm / n);
}

bool finiteVec(const geometry_msgs::Point &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool finiteVec(const geometry_msgs::Vector3 &v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}

class MockDroneTracker {
public:
    explicit MockDroneTracker(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("publish_rate", publish_rate_, 100.0);
        nh_.param("init_x", position_.x, 0.0);
        nh_.param("init_y", position_.y, 0.0);
        nh_.param("init_z", position_.z, 0.5);
        nh_.param("max_vel", max_vel_, 1.5);
        nh_.param("max_acc", max_acc_, 1.0);
        nh_.param("position_gain", position_gain_, 1.2);
        nh_.param("command_timeout", command_timeout_, 1.0);

        target_position_ = position_;

        cmd_sub_ = nh_.subscribe("/position_cmd", 20, &MockDroneTracker::cmdCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/Odom_high_freq", 50);
        status_pub_ = nh_.advertise<std_msgs::String>("/mock_drone/status", 5);
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &MockDroneTracker::timerCallback, this);
        last_update_time_ = ros::Time::now();
    }

private:
    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
        received_cmd_ = true;
        last_cmd_time_ = ros::Time::now();
        const bool finite = finiteVec(msg->position) &&
                            finiteVec(msg->velocity) &&
                            finiteVec(msg->acceleration) &&
                            std::isfinite(msg->yaw);
        command_finite_ = finite;
        if (!finite) {
            ROS_ERROR_THROTTLE(1.0, "[mock_drone_tracker] Non-finite /position_cmd received");
            return;
        }

        target_position_ = {msg->position.x, msg->position.y, msg->position.z};
        target_velocity_ = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
        target_yaw_ = msg->yaw;
    }

    void timerCallback(const ros::TimerEvent &) {
        const ros::Time now = ros::Time::now();
        double dt = (now - last_update_time_).toSec();
        last_update_time_ = now;
        if (dt <= 0.0 || dt > 0.2) {
            dt = 1.0 / publish_rate_;
        }

        const bool cmd_fresh = received_cmd_ && (now - last_cmd_time_).toSec() <= command_timeout_;
        if (cmd_fresh && command_finite_) {
            Vec3 desired_velocity = (target_position_ - position_) * position_gain_ + target_velocity_;
            desired_velocity = limitNorm(desired_velocity, max_vel_);
            Vec3 accel_cmd = limitNorm((desired_velocity - velocity_) * (1.0 / dt), max_acc_);
            velocity_ = limitNorm(velocity_ + accel_cmd * dt, max_vel_);
            position_ = position_ + velocity_ * dt;
            yaw_ = target_yaw_;
        }

        publishOdom(now);
        publishStatus(now, cmd_fresh);
    }

    void publishOdom(const ros::Time &stamp) {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "world";
        odom.child_frame_id = "mock_drone";
        odom.pose.pose.position.x = position_.x;
        odom.pose.pose.position.y = position_.y;
        odom.pose.pose.position.z = position_.z;
        odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
        odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
        odom.twist.twist.linear.x = velocity_.x;
        odom.twist.twist.linear.y = velocity_.y;
        odom.twist.twist.linear.z = velocity_.z;
        odom_pub_.publish(odom);
    }

    void publishStatus(const ros::Time &stamp, const bool cmd_fresh) {
        std_msgs::String status;
        const double error = norm(target_position_ - position_);
        std::ostringstream ss;
        ss << "stamp=" << stamp.toSec()
           << " ok=" << (command_finite_ ? "true" : "false")
           << " received_cmd=" << (received_cmd_ ? "true" : "false")
           << " cmd_fresh=" << (cmd_fresh ? "true" : "false")
           << " position=[" << position_.x << "," << position_.y << "," << position_.z << "]"
           << " target=[" << target_position_.x << "," << target_position_.y << "," << target_position_.z << "]"
           << " target_error=" << error;
        status.data = ss.str();
        status_pub_.publish(status);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher status_pub_;
    ros::Timer timer_;

    Vec3 position_;
    Vec3 velocity_;
    Vec3 target_position_;
    Vec3 target_velocity_;
    double yaw_{0.0};
    double target_yaw_{0.0};
    double publish_rate_{100.0};
    double max_vel_{1.5};
    double max_acc_{1.0};
    double position_gain_{1.2};
    double command_timeout_{1.0};
    bool received_cmd_{false};
    bool command_finite_{true};
    ros::Time last_cmd_time_{0.0};
    ros::Time last_update_time_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mock_drone_tracker");
    ros::NodeHandle nh("~");
    MockDroneTracker node(nh);
    ros::spin();
    return 0;
}
