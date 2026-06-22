#include <cmath>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

class GazeboCloudBridge {
public:
    explicit GazeboCloudBridge(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param<std::string>("input_topic", input_topic_, "/gazebo_lite/points");
        nh_.param<double>("publish_rate", publish_rate_, 5.0);
        nh_.param<double>("point_resolution", point_resolution_, 0.12);
        nh_.param<bool>("fallback_geometry_cloud", fallback_geometry_cloud_, true);
        nh_.param<std::string>("course_type", course_type_, "competition");

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 2);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/competition_sim/field_markers", 1, true);
        input_sub_ = nh_.subscribe(input_topic_, 2, &GazeboCloudBridge::cloudCallback, this);

        buildFallbackCloud();
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &GazeboCloudBridge::timerCallback, this);
    }

private:
    struct Box {
        double x;
        double y;
        double z;
        double sx;
        double sy;
        double sz;
    };

    struct Cylinder {
        double x;
        double y;
        double radius;
        double height;
    };

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
        last_input_time_ = ros::Time::now();
        have_input_cloud_ = true;
        sensor_msgs::PointCloud2 out = *msg;
        if (out.width == 0 || out.height == 0) {
            ROS_WARN_THROTTLE(2.0, "[gazebo_cloud_bridge] Gazebo input point cloud is empty");
        }
        if (out.header.frame_id != "world") {
            ROS_WARN_THROTTLE(5.0,
                              "[gazebo_cloud_bridge] Input cloud frame is '%s', republishing as world",
                              out.header.frame_id.c_str());
        }
        out.header.stamp = ros::Time::now();
        out.header.frame_id = "world";
        latest_cloud_ = out;
    }

    void timerCallback(const ros::TimerEvent &) {
        const bool input_fresh = have_input_cloud_ &&
                                 (ros::Time::now() - last_input_time_).toSec() < 1.0;
        if (input_fresh) {
            latest_cloud_.header.stamp = ros::Time::now();
            latest_cloud_.header.frame_id = "world";
            cloud_pub_.publish(latest_cloud_);
        } else if (fallback_geometry_cloud_) {
            ROS_WARN_THROTTLE(5.0,
                              "[gazebo_cloud_bridge] No fresh Gazebo ray cloud on %s; using Gazebo-lite geometry fallback cloud",
                              input_topic_.c_str());
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(fallback_cloud_, msg);
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "world";
            cloud_pub_.publish(msg);
        } else {
            ROS_WARN_THROTTLE(2.0, "[gazebo_cloud_bridge] No cloud published: input missing and fallback disabled");
        }

        stampAndPublishMarkers();
    }

    void buildFallbackCloud() {
        fallback_cloud_.clear();
        markers_.markers.clear();
        int id = 0;

        if (course_type_ == "obstacle_course") {
            buildObstacleCourseFallbackCloud(id);
        } else {
            buildCompetitionFallbackCloud(id);
        }

        fallback_cloud_.width = static_cast<uint32_t>(fallback_cloud_.points.size());
        fallback_cloud_.height = 1;
        fallback_cloud_.is_dense = true;
        fallback_cloud_.header.frame_id = "world";
        ROS_INFO_STREAM("[gazebo_cloud_bridge] Built " << course_type_ << " fallback cloud with "
                        << fallback_cloud_.points.size() << " points");
    }

    void buildCompetitionFallbackCloud(int &id) {
        addFieldMarker(id++, 5.0, 0.0, 12.0, 8.0);
        addBox({2.0, -2.6, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({2.8, -1.4, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({3.6, -2.5, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({4.4, -1.2, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({2.0, 2.6, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({2.8, 1.4, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({3.6, 2.5, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({4.4, 1.2, 1.0, 0.5, 0.5, 2.0}, id++);

        addCylinder({5.4, -2.8, 0.23, 1.8}, id++);
        addCylinder({6.0, -1.7, 0.23, 1.8}, id++);
        addCylinder({6.5, -2.5, 0.23, 1.8}, id++);
        addCylinder({5.5, 1.8, 0.23, 1.8}, id++);
        addCylinder({6.8, 2.8, 0.23, 1.8}, id++);

        addBox({7.5, -2.1, 1.0, 0.5, 0.5, 2.0}, id++);
        addBox({7.5, 2.1, 1.0, 0.5, 0.5, 2.0}, id++);

        addBox({9.2, -0.75, 0.85, 0.22, 0.22, 1.7}, id++);
        addBox({9.2, 0.75, 0.85, 0.22, 0.22, 1.7}, id++);
        addBox({9.2, 0.0, 1.72, 0.22, 1.72, 0.22}, id++);
    }

    void buildObstacleCourseFallbackCloud(int &id) {
        addFieldMarker(id++, 4.0, 0.0, 10.0, 6.0);
        addBox({4.0, 0.0, 1.0, 1.0, 2.0, 2.0}, id++);
    }

    void addFieldMarker(const int id, const double x, const double y,
                        const double sx, const double sy) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "field";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.01;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = sx;
        marker.scale.y = sy;
        marker.scale.z = 0.02;
        marker.color.r = 0.2;
        marker.color.g = 0.2;
        marker.color.b = 0.2;
        marker.color.a = 0.25;
        markers_.markers.push_back(marker);
    }

    void addBox(const Box &box, const int marker_id) {
        const double min_x = box.x - box.sx / 2.0;
        const double max_x = box.x + box.sx / 2.0;
        const double min_y = box.y - box.sy / 2.0;
        const double max_y = box.y + box.sy / 2.0;
        const double min_z = std::max(0.0, box.z - box.sz / 2.0);
        const double max_z = box.z + box.sz / 2.0;
        for (double x = min_x; x <= max_x; x += point_resolution_) {
            for (double y = min_y; y <= max_y; y += point_resolution_) {
                for (double z = min_z; z <= max_z; z += point_resolution_) {
                    const bool surface = near(x, min_x) || near(x, max_x) ||
                                         near(y, min_y) || near(y, max_y) ||
                                         near(z, min_z) || near(z, max_z);
                    if (surface) {
                        addPoint(x, y, z);
                    }
                }
            }
        }
        addBoxMarker(box, marker_id);
    }

    void addCylinder(const Cylinder &cylinder, const int marker_id) {
        for (double z = 0.0; z <= cylinder.height; z += point_resolution_) {
            for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.22) {
                addPoint(cylinder.x + cylinder.radius * std::cos(angle),
                         cylinder.y + cylinder.radius * std::sin(angle),
                         z);
            }
        }
        addCylinderMarker(cylinder, marker_id);
    }

    bool near(const double value, const double boundary) const {
        return std::fabs(value - boundary) < point_resolution_ * 0.51;
    }

    void addPoint(const double x, const double y, const double z) {
        pcl::PointXYZI point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        point.intensity = 1.0f;
        fallback_cloud_.points.push_back(point);
    }

    void addBoxMarker(const Box &box, const int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "gazebo_lite_obstacles";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = box.x;
        marker.pose.position.y = box.y;
        marker.pose.position.z = box.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = box.sx;
        marker.scale.y = box.sy;
        marker.scale.z = box.sz;
        marker.color.r = 0.9;
        marker.color.g = 0.45;
        marker.color.b = 0.1;
        marker.color.a = 0.55;
        markers_.markers.push_back(marker);
    }

    void addCylinderMarker(const Cylinder &cylinder, const int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "gazebo_lite_obstacles";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cylinder.x;
        marker.pose.position.y = cylinder.y;
        marker.pose.position.z = cylinder.height / 2.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = cylinder.radius * 2.0;
        marker.scale.y = cylinder.radius * 2.0;
        marker.scale.z = cylinder.height;
        marker.color.r = 0.1;
        marker.color.g = 0.65;
        marker.color.b = 0.25;
        marker.color.a = 0.65;
        markers_.markers.push_back(marker);
    }

    void stampAndPublishMarkers() {
        for (auto &marker : markers_.markers) {
            marker.header.stamp = ros::Time::now();
        }
        marker_pub_.publish(markers_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
    std::string input_topic_;
    std::string course_type_{"competition"};
    double publish_rate_{5.0};
    double point_resolution_{0.12};
    bool fallback_geometry_cloud_{true};
    bool have_input_cloud_{false};
    ros::Time last_input_time_;
    sensor_msgs::PointCloud2 latest_cloud_;
    pcl::PointCloud<pcl::PointXYZI> fallback_cloud_;
    visualization_msgs::MarkerArray markers_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_cloud_bridge");
    ros::NodeHandle nh("~");
    GazeboCloudBridge node(nh);
    ros::spin();
    return 0;
}
