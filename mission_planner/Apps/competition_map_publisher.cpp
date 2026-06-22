#include <cmath>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

class CompetitionMapPublisher {
public:
    explicit CompetitionMapPublisher(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("field_length", field_length_, 12.0);
        nh_.param("field_width", field_width_, 8.0);
        nh_.param("field_height", field_height_, 2.0);
        nh_.param("publish_rate", publish_rate_, 5.0);
        nh_.param("point_resolution", point_resolution_, 0.12);
        nh_.param("zone1_box_count", zone1_box_count_, 8);
        nh_.param("zone2_tree_count", zone2_tree_count_, 5);
        nh_.param("zone3_moving_count", zone3_moving_count_, 2);

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/competition_map/markers", 1, true);

        buildMap();
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &CompetitionMapPublisher::timerCallback, this);
    }

private:
    struct Box {
        double x;
        double y;
        double z;
        double sx;
        double sy;
        double sz;
        std::string ns;
    };

    struct Cylinder {
        double x;
        double y;
        double radius;
        double height;
        std::string ns;
    };

    void buildMap() {
        cloud_.clear();
        markers_.markers.clear();
        int marker_id = 0;

        addBoundaryMarkers(marker_id);

        const std::vector<std::pair<double, double>> box_centers = {
            {2.0, -2.6}, {2.8, -1.4}, {3.6, -2.5}, {4.4, -1.2},
            {2.0, 2.6}, {2.8, 1.4}, {3.6, 2.5}, {4.4, 1.2},
        };
        for (int i = 0; i < zone1_box_count_ && i < static_cast<int>(box_centers.size()); ++i) {
            const auto &p = box_centers[i];
            addBox({p.first, p.second, field_height_ / 2.0, 0.5, 0.5, field_height_, "zone1_box"}, marker_id);
        }

        const std::vector<std::pair<double, double>> trees = {
            {5.4, -2.8}, {6.0, -1.7}, {6.5, -2.5}, {5.5, 1.8}, {6.8, 2.8},
        };
        for (int i = 0; i < zone2_tree_count_ && i < static_cast<int>(trees.size()); ++i) {
            const auto &p = trees[i];
            addCylinder({p.first, p.second, 0.23, 1.8, "zone2_tree"}, marker_id);
        }

        const std::vector<std::pair<double, double>> moving_slots = {
            {7.5, -2.1}, {7.5, 2.1},
        };
        for (int i = 0; i < zone3_moving_count_ && i < static_cast<int>(moving_slots.size()); ++i) {
            const auto &p = moving_slots[i];
            addBox({p.first, p.second, field_height_ / 2.0, 0.5, 0.5, field_height_, "zone3_static_moving_slot"}, marker_id);
        }

        addGate(9.2, 0.0, marker_id);

        cloud_.width = static_cast<uint32_t>(cloud_.points.size());
        cloud_.height = 1;
        cloud_.is_dense = true;
        cloud_.header.frame_id = "world";

        ROS_INFO_STREAM("[competition_map_publisher] Generated "
                        << cloud_.points.size() << " map points");
    }

    void addBoundaryMarkers(int &marker_id) {
        addWireBoxMarker(marker_id++, "field_boundary", 0.0, 0.0, field_height_ / 2.0,
                         field_length_, field_width_, field_height_, 0.2, 0.8, 1.0, 0.25);
        addWireBoxMarker(marker_id++, "takeoff_zone", 0.0, 0.0, 0.02,
                         1.2, 1.2, 0.04, 0.1, 1.0, 0.2, 0.55);
    }

    void addGate(const double x, const double y, int &marker_id) {
        addBox({x, y - 0.75, 0.85, 0.22, 0.22, 1.7, "zone4_gate"}, marker_id);
        addBox({x, y + 0.75, 0.85, 0.22, 0.22, 1.7, "zone4_gate"}, marker_id);
        addBox({x, y, 1.72, 0.22, 1.72, 0.22, "zone4_gate"}, marker_id);
    }

    void addBox(const Box &box, int &marker_id) {
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

        addCubeMarker(marker_id++, box.ns, box.x, box.y, box.z, box.sx, box.sy, box.sz,
                      1.0, 0.45, 0.1, 0.55);
    }

    void addCylinder(const Cylinder &cylinder, int &marker_id) {
        for (double z = 0.0; z <= cylinder.height; z += point_resolution_) {
            for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.22) {
                addPoint(cylinder.x + cylinder.radius * std::cos(angle),
                         cylinder.y + cylinder.radius * std::sin(angle),
                         z);
            }
        }
        addCylinderMarker(marker_id++, cylinder.ns, cylinder.x, cylinder.y, cylinder.height / 2.0,
                          cylinder.radius * 2.0, cylinder.height, 0.0, 0.65, 0.25, 0.65);
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
        cloud_.points.push_back(point);
    }

    void addCubeMarker(const int id, const std::string &ns, const double x, const double y, const double z,
                       const double sx, const double sy, const double sz,
                       const double r, const double g, const double b, const double a) {
        visualization_msgs::Marker marker;
        fillMarkerBase(marker, id, ns, visualization_msgs::Marker::CUBE);
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = sx;
        marker.scale.y = sy;
        marker.scale.z = sz;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        markers_.markers.push_back(marker);
    }

    void addCylinderMarker(const int id, const std::string &ns, const double x, const double y, const double z,
                           const double diameter, const double height,
                           const double r, const double g, const double b, const double a) {
        visualization_msgs::Marker marker;
        fillMarkerBase(marker, id, ns, visualization_msgs::Marker::CYLINDER);
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = height;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        markers_.markers.push_back(marker);
    }

    void addWireBoxMarker(const int id, const std::string &ns, const double x, const double y, const double z,
                          const double sx, const double sy, const double sz,
                          const double r, const double g, const double b, const double a) {
        visualization_msgs::Marker marker;
        fillMarkerBase(marker, id, ns, visualization_msgs::Marker::CUBE);
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = sx;
        marker.scale.y = sy;
        marker.scale.z = sz;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        markers_.markers.push_back(marker);
    }

    void fillMarkerBase(visualization_msgs::Marker &marker, const int id,
                        const std::string &ns, const int type) const {
        marker.header.frame_id = "world";
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.0);
    }

    void timerCallback(const ros::TimerEvent &) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        cloud_pub_.publish(cloud_msg);

        for (auto &marker : markers_.markers) {
            marker.header.stamp = ros::Time::now();
        }
        marker_pub_.publish(markers_);
    }

    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    visualization_msgs::MarkerArray markers_;

    double field_length_{12.0};
    double field_width_{8.0};
    double field_height_{2.0};
    double publish_rate_{5.0};
    double point_resolution_{0.12};
    int zone1_box_count_{8};
    int zone2_tree_count_{5};
    int zone3_moving_count_{2};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "competition_map_publisher");
    ros::NodeHandle nh("~");
    CompetitionMapPublisher node(nh);
    ros::spin();
    return 0;
}
