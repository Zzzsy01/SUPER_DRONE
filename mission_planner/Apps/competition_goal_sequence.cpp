#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class CompetitionGoalSequence {
public:
    explicit CompetitionGoalSequence(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("goal_topic", goal_topic_, std::string("/planning/click_goal"));
        nh_.param("frame_id", frame_id_, std::string("world"));
        nh_.param("reach_radius", default_reach_radius_, 0.5);
        nh_.param("switch_delay", switch_delay_, 1.0);
        nh_.param("initial_delay", initial_delay_, 2.0);
        loadGoals();

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/competition_sim/current_goal", 1, true);
        odom_sub_ = nh_.subscribe("/Odom_high_freq", 20, &CompetitionGoalSequence::odomCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.05), &CompetitionGoalSequence::timerCallback, this);
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
        goals_.clear();
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

    void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
        have_odom_ = true;
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;
    }

    void timerCallback(const ros::TimerEvent &) {
        if (mission_complete_ || !have_odom_) {
            return;
        }

        const ros::Time now = ros::Time::now();
        if (!started_) {
            if ((now - start_time_).toSec() < initial_delay_) {
                return;
            }
            publishCurrentGoal();
            started_ = true;
            return;
        }

        if (waiting_to_switch_) {
            if ((now - reached_time_).toSec() >= switch_delay_) {
                ++current_goal_idx_;
                if (current_goal_idx_ >= goals_.size()) {
                    mission_complete_ = true;
                    ROS_INFO("[competition_goal_sequence] mission complete");
                    return;
                }
                waiting_to_switch_ = false;
                publishCurrentGoal();
            }
            return;
        }

        const Goal &goal = goals_[current_goal_idx_];
        const double dx = odom_x_ - goal.x;
        const double dy = odom_y_ - goal.y;
        const double dz = odom_z_ - goal.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= goal.reach_radius) {
            waiting_to_switch_ = true;
            reached_time_ = now;
            ROS_INFO_STREAM("[competition_goal_sequence] reached goal "
                            << current_goal_idx_ + 1 << "/" << goals_.size()
                            << ", switching after " << switch_delay_ << " s");
        }
    }

    void publishCurrentGoal() {
        const Goal &goal = goals_[current_goal_idx_];
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        msg.pose.position.x = goal.x;
        msg.pose.position.y = goal.y;
        msg.pose.position.z = goal.z;
        msg.pose.orientation.w = 1.0;
        goal_pub_.publish(msg);
        publishCurrentGoalMarker(goal);

        ROS_INFO_STREAM("[competition_goal_sequence] published goal "
                        << current_goal_idx_ + 1 << "/" << goals_.size()
                        << ": [" << goal.x << ", " << goal.y << ", " << goal.z << "]");
    }

    void publishCurrentGoalMarker(const Goal &goal) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "competition_current_goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goal.x;
        marker.pose.position.y = goal.y;
        marker.pose.position.z = goal.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = goal.reach_radius * 2.0;
        marker.scale.y = goal.reach_radius * 2.0;
        marker.scale.z = goal.reach_radius * 2.0;
        marker.color.r = 0.1;
        marker.color.g = 1.0;
        marker.color.b = 0.25;
        marker.color.a = 0.75;
        marker.lifetime = ros::Duration(0.0);
        marker_pub_.publish(marker);
    }

    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    std::vector<Goal> goals_;

    std::string goal_topic_{"/planning/click_goal"};
    std::string frame_id_{"world"};
    double default_reach_radius_{0.5};
    double switch_delay_{1.0};
    double initial_delay_{2.0};
    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_z_{0.0};
    bool have_odom_{false};
    bool started_{false};
    bool waiting_to_switch_{false};
    bool mission_complete_{false};
    size_t current_goal_idx_{0};
    ros::Time start_time_;
    ros::Time reached_time_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "competition_goal_sequence");
    ros::NodeHandle nh("~");
    CompetitionGoalSequence node(nh);
    ros::spin();
    return 0;
}
