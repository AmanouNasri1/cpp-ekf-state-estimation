#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp" // New Include
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "ekf_project/diff_drive_kinematics.hpp"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct MapLandmark {
    int id;
    double x;
    double y;
};

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher_node")
    {
        kinematics_ = std::make_unique<DiffDriveKinematics>();

        // 1. DEFINE THE MAP (Ground Truth positions from SDF)
        // Cylinder 1: 2, 0
        // Cylinder 2: 2, 2
        // Cylinder 3: 0, -2
        map_landmarks_.push_back({1, 2.0, 0.0});
        map_landmarks_.push_back({2, 2.0, 2.0});
        map_landmarks_.push_back({3, 0.0, -2.0});

        // Subscribers
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OdomPublisher::vel_callback, this, _1));
        
        // !!! NEW SUBSCRIBER FOR LANDMARKS !!!
        // Define QoS profile matching the publisher
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

            // New Subscriber for Landmarks (Must match publisher QoS)
        landmark_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/landmarks", 
            qos_profile, // <-- Use the defined QoS profile here
            std::bind(&OdomPublisher::landmark_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_est", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            20ms, std::bind(&OdomPublisher::timer_callback, this));

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "EKF Node Started. Listening for Landmarks...");
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_v_ = msg->linear.x;
        current_w_ = msg->angular.z;
    }

    // !!! THIS IS THE SENSOR FUSION LOGIC !!!
    void landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        auto state = kinematics_->getState(); // Current Estimate [x, y, theta]
        
        // Loop through every observed landmark
        for (const auto& obs_pose : msg->poses) {
            
            // 1. Convert Observation to Range/Bearing
            // The landmark_extractor sends x,y relative to robot (base_link)
            double r = std::sqrt(obs_pose.position.x*obs_pose.position.x + obs_pose.position.y*obs_pose.position.y);
            double b = std::atan2(obs_pose.position.y, obs_pose.position.x);

            // 2. DATA ASSOCIATION (Nearest Neighbor)
            // We need to figure out: Which Map Landmark is this?
            
            int best_id = -1;
            double min_dist = 1000.0; // Init with large number

            // Calculate estimated global position of this observation
            double obs_global_x = state(0) + r * std::cos(state(2) + b);
            double obs_global_y = state(1) + r * std::sin(state(2) + b);

            for (const auto& lm : map_landmarks_) {
                double dx = obs_global_x - lm.x;
                double dy = obs_global_y - lm.y;
                double dist = std::sqrt(dx*dx + dy*dy);

                if (dist < min_dist) {
                    min_dist = dist;
                    best_id = lm.id;
                }
            }

            // 3. GATING (Safety Check)
            if (best_id != -1 && min_dist < 1.0) {
                // Find the map coordinates again
                double map_x = 0; 
                double map_y = 0;
                for(auto& m : map_landmarks_) { if(m.id == best_id) { map_x = m.x; map_y = m.y; break; } }

                // 4. CALL THE EKF UPDATE STEP
                // "I saw a landmark at (r,b), and I know it is actually at (map_x, map_y). Fix me!"
                kinematics_->update(r, b, map_x, map_y);
                
                // Debug log (Optional)
                // RCLCPP_INFO(this->get_logger(), "Updated using Landmark %d", best_id);
            }
        }
    }

    void timer_callback()
    {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt > 0.1) dt = 0.02; 

        // Prediction Step
        kinematics_->integrate(current_v_, current_w_, dt);
        
        auto state = kinematics_->getState();
        auto cov   = kinematics_->getCovariance();

        // Publish TF
        tf2::Quaternion q;
        q.setRPY(0, 0, state(2)); 

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link_est";
        t.transform.translation.x = state(0);
        t.transform.translation.y = state(1);
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // Publish Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link_est";
        odom_msg.pose.pose.position.x = state(0);
        odom_msg.pose.pose.position.y = state(1);
        odom_msg.pose.pose.orientation = t.transform.rotation;

        // Fill Covariance (6x6) from Eigen (3x3)
        // 0=X, 1=Y, 5=Theta
        for(int i=0; i<36; i++) odom_msg.pose.covariance[i] = 0.0;
        odom_msg.pose.covariance[0] = cov(0,0); odom_msg.pose.covariance[1] = cov(0,1); odom_msg.pose.covariance[5] = cov(0,2);
        odom_msg.pose.covariance[6] = cov(1,0); odom_msg.pose.covariance[7] = cov(1,1); odom_msg.pose.covariance[11] = cov(1,2);
        odom_msg.pose.covariance[30]= cov(2,0); odom_msg.pose.covariance[31]= cov(2,1); odom_msg.pose.covariance[35]= cov(2,2);

        odom_pub_->publish(odom_msg);
    }

    std::unique_ptr<DiffDriveKinematics> kinematics_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_sub_; // New sub
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    double current_v_ = 0.0;
    double current_w_ = 0.0;
    std::vector<MapLandmark> map_landmarks_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}