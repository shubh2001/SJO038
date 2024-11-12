// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include <cmath>
// #include <sensor_msgs/msg/laser_scan.hpp>

// class FollowerBot : public rclcpp::Node {
// public:
//     FollowerBot() : Node("follower_bot"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
//         // Publisher for follower's velocity commands
//         velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/follower/cmd_vel", 10);
//         scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "follower/scan", 10, std::bind(&FollowerBot::scan_callback, this, std::placeholders::_1));

        
//         // Timer to update the follower's movement
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),
//             std::bind(&FollowerBot::followRobot1, this));
//     }

// private:

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;

//     bool obstacle = false;
//     bool turn = false;
//     float distanceThreshold = 0.0;
//     float turnDir = 1.0;

//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         obstacle = false;
//         distanceThreshold = msg->range_max;

//         int num_ranges = msg->ranges.size();
//         float angle_increment = 360.0 / num_ranges;

//         int start_index_1 = 0;
//         int end_index_1 = 30 / angle_increment;
//         int start_index_2 = num_ranges - 30 / angle_increment;
//         int end_index_2 = num_ranges - 1;

//         for (int i = start_index_1; i <= end_index_1; ++i) {
//             float range = msg->ranges[i];
//             if (std::isfinite(range) && range < 0.4) {
//                 obstacle = true;
//                 distanceThreshold = range;
//                 break;
//             }
//         }

//         if (!obstacle) {
//             for (int i = start_index_2; i <= end_index_2; ++i) {
//                 float range = msg->ranges[i];
//                 if (std::isfinite(range) && range < 0.4) {
//                     obstacle = true;
//                     distanceThreshold = range;
//                     break;
//                 }
//             }
//         }
//     }

//     void followRobot1() {
//         geometry_msgs::msg::TransformStamped transformStamped;
//         try {
//             // Lookup transform from follower to robot1
//             transformStamped = tf_buffer_.lookupTransform("follower/base_link", "robot1/base_link", tf2::TimePointZero);

//             // double dx = transformStamped.transform.translation.x;
//             // double dy = transformStamped.transform.translation.y;
//             // double distance = std::sqrt(dx * dx + dy * dy);
//             // double angle = std::atan2(dy, dx);
//             double distance = std::sqrt(std::pow(transformStamped.transform.translation.x, 2) +
//                                         std::pow(transformStamped.transform.translation.y, 2));
//             double angle = std::atan2(transformStamped.transform.translation.y, 
//                                                 transformStamped.transform.translation.x);

//             // Create velocity command
//             geometry_msgs::msg::Twist cmd_vel;
//             RCLCPP_INFO(this->get_logger(), "This is an info message %f", distance);
//             if(obstacle){
//                 turnDir = (angle > 0) ? 1.0 : -1.0;
//                 cmd_vel.angular.z = 1 * turnDir;
//                 cmd_vel.linear.x = 0.0;
//             }
//             else if (distance < 0.4) { // Close enough to stop
//                 cmd_vel.linear.x = 0.0;
//                 cmd_vel.angular.z = 0.0;
//             } else {
//                 cmd_vel.linear.x = std::min(0.5, distance); // Cap linear speed
//                 cmd_vel.angular.z = angle;                  // Direction towards robot1
//             }

//             // Publish the command
//             velocity_publisher_->publish(cmd_vel);
//         } catch (const tf2::TransformException &ex) {
//             RCLCPP_WARN(this->get_logger(), "Could not transform follower/odom to robot1/odom: %s", ex.what());
//         }
//     }
// };

// int main(int argc, char * argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FollowerBot>());
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>

class FollowerBot : public rclcpp::Node {
public:
    FollowerBot() : Node("follower_bot"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Publisher for follower's velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/follower/cmd_vel", 10);
        
        // Laser scan subscription for obstacle avoidance
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/follower/scan", 10, std::bind(&FollowerBot::scan_callback, this, std::placeholders::_1));
        
        // Timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FollowerBot::controlLoop, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool obstacle_in_front = false;
    bool obstacle_left = false;
    bool obstacle_right = false;
    double obstacle_distance_ = 0.4; // Distance threshold for obstacles
    double linear_speed_ = 0.2;
    double angular_speed_ = 0.5;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd_vel;

        obstacle_in_front = false;
        obstacle_left = false;
        obstacle_right = false;

        // Check front, left, and right regions for obstacles
        float front = std::min(
            *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 10),
            *std::min_element(msg->ranges.end() - 10, msg->ranges.end())
        );

        float left = *std::min_element(msg->ranges.begin() + 40, msg->ranges.begin() + 80);
        float right = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 300);

        obstacle_in_front = front < obstacle_distance_;
        obstacle_left = left < obstacle_distance_;
        obstacle_right = right < obstacle_distance_;

    }

    void controlLoop() {
        geometry_msgs::msg::Twist cmd_vel;
        
        RCLCPP_INFO(this->get_logger(), "Front: %.2d, Left: %.2d, Right: %.2d", obstacle_in_front, obstacle_left, obstacle_right);


        if (obstacle_in_front || obstacle_left || obstacle_right) {
            // Obstacle avoidance logic
            if (!obstacle_left && !obstacle_right && !obstacle_in_front) {
                // Move forward
                cmd_vel.linear.x = linear_speed_;
                cmd_vel.angular.z = 0.0;
            } else if (!obstacle_left && obstacle_in_front && !obstacle_right){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -angular_speed_;

            } else if (!obstacle_left && obstacle_in_front && obstacle_right) {
                // Turn left
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_speed_;
            } else if (obstacle_left && !obstacle_in_front && !obstacle_right) {
                // Turn right
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -angular_speed_;
            } else if (obstacle_left && obstacle_in_front && !obstacle_right) {
                // Turn right
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -angular_speed_;
            } else if (obstacle_left && obstacle_in_front && obstacle_right) {
                // Reverse or stop
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_speed_;
            } else if (!obstacle_left && !obstacle_in_front && obstacle_right) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_speed_;
            } 
        } 
        else {
            // No obstacle, follow robot1
            try {
                auto transformStamped = tf_buffer_.lookupTransform("follower/base_link", "robot1/base_link", tf2::TimePointZero);
                double distance = std::sqrt(std::pow(transformStamped.transform.translation.x, 2) +
                                            std::pow(transformStamped.transform.translation.y, 2));
                double angle = std::atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);

                if (distance > 0.4) {
                    cmd_vel.linear.x = std::min(0.5, distance); // Cap linear speed
                    cmd_vel.angular.z = angle;                  // Direction towards robot1
                } else {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                }
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform follower/base_link to robot1/base_link: %s", ex.what());
            }
        }

        // Publish the command
        velocity_publisher_->publish(cmd_vel);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowerBot>());
    rclcpp::shutdown();
    return 0;
}

