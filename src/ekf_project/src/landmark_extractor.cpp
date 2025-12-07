#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <vector>
#include <cmath>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class LandmarkExtractor : public rclcpp::Node
{
public:
    LandmarkExtractor() : Node("landmark_extractor")
    {
        // QoS for PoseArray (sensor-like, fast, allowed to drop)
        auto landmarks_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // QoS for MarkerArray (RViz2 requires RELIABLE + TRANSIENT_LOCAL)
        auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                            .reliable()
                            .transient_local();

        // Subscriber to LiDAR scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LandmarkExtractor::scan_callback, this, std::placeholders::_1));

        // Publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/landmark_markers",
            marker_qos);

        landmarks_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/landmarks",
            landmarks_qos);

        // Timer (10 Hz) to publish stored data
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&LandmarkExtractor::timer_callback, this));

        last_markers_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
        last_poses_ = std::make_shared<geometry_msgs::msg::PoseArray>();

        RCLCPP_INFO(this->get_logger(), "Landmark Extractor Started and throttled to 10Hz!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::vector<std::pair<float, float>>> clusters;
        std::vector<std::pair<float, float>> current_cluster;

        // --- Cluster points ---
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];

            if (std::isinf(r) || r > msg->range_max || r < msg->range_min)
                continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            if (current_cluster.empty()) {
                current_cluster.push_back({x, y});
            } else {
                float last_x = current_cluster.back().first;
                float last_y = current_cluster.back().second;
                float dist_sq = (x - last_x)*(x - last_x) + (y - last_y)*(y - last_y);

                if (dist_sq < 0.1 * 0.1) {
                    current_cluster.push_back({x, y});
                } else {
                    if (current_cluster.size() > 3)
                        clusters.push_back(current_cluster);

                    current_cluster.clear();
                    current_cluster.push_back({x, y});
                }
            }
        }

        if (current_cluster.size() > 3)
            clusters.push_back(current_cluster);

        visualization_msgs::msg::MarkerArray markers;
        geometry_msgs::msg::PoseArray pose_array_msg;

        pose_array_msg.header.frame_id = "base_link";
        pose_array_msg.header.stamp = this->now();

        int id = 0;

        for (const auto& cluster : clusters) {
            float dx = cluster.back().first - cluster.front().first;
            float dy = cluster.back().second - cluster.front().second;
            float width = std::sqrt(dx*dx + dy*dy);

            if (width < 0.15 || width > 0.45)
                continue;

            float sum_x = 0, sum_y = 0;
            for (const auto& p : cluster) {
                sum_x += p.first;
                sum_y += p.second;
            }

            float avg_x = sum_x / cluster.size();
            float avg_y = sum_y / cluster.size();

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "landmarks";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = avg_x;
            marker.pose.position.y = avg_y;
            marker.pose.position.z = 0.5;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 1.0;
            marker.color.a = 1.0;
            marker.color.g = 1.0;
            markers.markers.push_back(marker);

            geometry_msgs::msg::Pose p;
            p.position.x = avg_x;
            p.position.y = avg_y;
            pose_array_msg.poses.push_back(p);
        }

        *last_markers_ = markers;
        *last_poses_ = pose_array_msg;
    }

    void timer_callback()
    {
        if (last_markers_->markers.empty()) {
            visualization_msgs::msg::MarkerArray delete_msg;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.action = visualization_msgs::msg::Marker::DELETEALL;
            delete_msg.markers.push_back(marker);
            marker_pub_->publish(delete_msg);

            landmarks_pub_->publish(*last_poses_);
            return;
        }

        marker_pub_->publish(*last_markers_);
        landmarks_pub_->publish(*last_poses_);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<visualization_msgs::msg::MarkerArray> last_markers_;
    std::shared_ptr<geometry_msgs::msg::PoseArray> last_poses_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkExtractor>());
    rclcpp::shutdown();
    return 0;
}
