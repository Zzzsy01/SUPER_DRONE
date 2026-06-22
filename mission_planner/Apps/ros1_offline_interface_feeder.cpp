#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

class OfflineInterfaceFeeder {
public:
    explicit OfflineInterfaceFeeder(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<double>("odom_rate", odom_rate_, 50.0);
        nh_.param<double>("cloud_rate", cloud_rate_, 5.0);
        nh_.param<double>("goal_delay", goal_delay_, 3.0);
        nh_.param<double>("goal_x", goal_x_, 2.0);
        nh_.param<double>("goal_y", goal_y_, 0.0);
        nh_.param<double>("goal_z", goal_z_, 1.0);
        nh_.param<bool>("publish_far_obstacles", publish_far_obstacles_, true);

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/Odom_high_freq", 20);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 5);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planning/click_goal", 1, true);

        odom_timer_ = nh_.createTimer(ros::Duration(1.0 / odom_rate_),
                                      &OfflineInterfaceFeeder::publishOdom, this);
        cloud_timer_ = nh_.createTimer(ros::Duration(1.0 / cloud_rate_),
                                       &OfflineInterfaceFeeder::publishCloud, this);
        goal_timer_ = nh_.createTimer(ros::Duration(goal_delay_),
                                      &OfflineInterfaceFeeder::publishGoalOnce, this, true);
    }

private:
    void publishOdom(const ros::TimerEvent &) const {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = "offline_drone";
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 1.0;
        odom.pose.pose.orientation.w = 1.0;
        odom_pub_.publish(odom);
    }

    void publishCloud(const ros::TimerEvent &) const {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.header.frame_id = frame_id_;

        if (publish_far_obstacles_) {
            const float z = 1.0f;
            const std::vector<pcl::PointXYZI> points = {
                makePoint(5.0f, 3.0f, z),
                makePoint(5.0f, -3.0f, z),
                makePoint(6.0f, 0.0f, z),
                makePoint(-5.0f, 3.0f, z),
                makePoint(-5.0f, -3.0f, z),
            };
            cloud.points.insert(cloud.points.end(), points.begin(), points.end());
            cloud.width = static_cast<uint32_t>(cloud.points.size());
            cloud.height = 1;
            cloud.is_dense = true;
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        cloud_pub_.publish(msg);
    }

    void publishGoalOnce(const ros::TimerEvent &) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = frame_id_;
        goal.pose.position.x = goal_x_;
        goal.pose.position.y = goal_y_;
        goal.pose.position.z = goal_z_;
        goal.pose.orientation.w = 1.0;
        goal_pub_.publish(goal);
        ROS_INFO_STREAM("[offline_interface_feeder] Published /planning/click_goal: ["
                        << goal_x_ << ", " << goal_y_ << ", " << goal_z_ << "]");
    }

    static pcl::PointXYZI makePoint(const float x, const float y, const float z) {
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 1.0f;
        return point;
    }

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher goal_pub_;
    ros::Timer odom_timer_;
    ros::Timer cloud_timer_;
    ros::Timer goal_timer_;

    std::string frame_id_;
    double odom_rate_{50.0};
    double cloud_rate_{5.0};
    double goal_delay_{3.0};
    double goal_x_{2.0};
    double goal_y_{0.0};
    double goal_z_{1.0};
    bool publish_far_obstacles_{true};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "offline_interface_feeder");
    ros::NodeHandle nh("~");

    OfflineInterfaceFeeder feeder(nh);
    ros::spin();
    return 0;
}
