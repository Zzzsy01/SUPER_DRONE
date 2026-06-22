#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class GoalFeeder {
public:
    explicit GoalFeeder(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param<std::string>("goal_topic", goal_topic_, "/planning/click_goal");
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<double>("goal_delay", goal_delay_, 3.0);
        nh_.param<double>("goal_x", goal_x_, 2.0);
        nh_.param<double>("goal_y", goal_y_, 0.0);
        nh_.param<double>("goal_z", goal_z_, 1.0);

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
        goal_timer_ = nh_.createTimer(ros::Duration(goal_delay_),
                                      &GoalFeeder::publishGoalOnce,
                                      this,
                                      true);
    }

private:
    void publishGoalOnce(const ros::TimerEvent &) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = frame_id_;
        goal.pose.position.x = goal_x_;
        goal.pose.position.y = goal_y_;
        goal.pose.position.z = goal_z_;
        goal.pose.orientation.w = 1.0;
        goal_pub_.publish(goal);

        ROS_INFO_STREAM("[goal_feeder] Published " << goal_topic_ << ": ["
                        << goal_x_ << ", " << goal_y_ << ", " << goal_z_
                        << "] in frame " << frame_id_);
    }

    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Timer goal_timer_;

    std::string goal_topic_;
    std::string frame_id_;
    double goal_delay_{3.0};
    double goal_x_{2.0};
    double goal_y_{0.0};
    double goal_z_{1.0};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_feeder");
    ros::NodeHandle nh("~");

    GoalFeeder feeder(nh);
    ros::spin();
    return 0;
}
