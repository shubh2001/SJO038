#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cstdlib> // for rand()
#include <algorithm> // for std::min_element
#include <cmath> // for atan2 and other math functions
#include "nav_msgs/msg/odometry.hpp"



class WanderNode : public rclcpp::Node {
public:
    WanderNode() : Node("wander_node") {
        // Declare and get the namespace parameter with a default value
        this->declare_parameter<std::string>("robot_namespace", "robot1");
        this->declare_parameter<double>("goal_x", 2.0);
        this->declare_parameter<double>("goal_y", 2.0);

        this->get_parameter("goal_x", goal_x_);
        this->get_parameter("goal_y", goal_y_);
        this->get_parameter("robot_namespace", robot_namespace_);

        linear_speed_ = 0.2;     // Forward speed
        angular_speed_ = 0.5;    // Turning speed
        obstacle_distance_ = 0.6;
        
        // Construct topic names with the namespace
        std::string laser_topic = "/" + robot_namespace_ + "/scan";
        std::string cmd_vel_topic = "/" + robot_namespace_ + "/cmd_vel";
        std::string odom_topic = "/" + robot_namespace_ + "/odom";

        // Create the subscriptions and publishers with the namespace
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic, 10, std::bind(&WanderNode::laser_callback, this, std::placeholders::_1)
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&WanderNode::odom_callback, this, std::placeholders::_1)
        );
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        // Analyze ranges to detect obstacles
        // bool obstacle_in_front = false;
        // bool obstacle_left = false;
        // bool obstacle_right = false;

        auto scan_data = msg->ranges;

        // Define the regions with specified indices
        float front = std::min(
            *std::min_element(scan_data.begin(), scan_data.begin() + 10),
            *std::min_element(scan_data.end() - 10, scan_data.end())
        );

        float left = *std::min_element(scan_data.begin() + 40, scan_data.begin() + 80);
        float right = *std::min_element(scan_data.begin() + 260, scan_data.begin() + 300);

        
        int front_obstacle = front < obstacle_distance_ ? 1 : 0;
        int left_obstacle = left < obstacle_distance_ ? 1 : 0;
        int right_obstacle = (right < obstacle_distance_) ? 1 : 0;

        // Calculate the angle to the goal
        float angle_to_goal = atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        float angle_difference = angle_to_goal - current_yaw_;
        while (angle_difference > M_PI) angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI) angle_difference += 2 * M_PI;


        RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f", front, left, right);
        RCLCPP_INFO(this->get_logger(), "Obstacles Front: %d, Left: %d, Right: %d", front_obstacle, left_obstacle, right_obstacle);

        if (left_obstacle == 0 && front_obstacle == 0 && right_obstacle == 0) {
            // Move forward
            RCLCPP_INFO(this->get_logger(), "Moving Forward");
            cmd_vel_msg.linear.x = 0.2;
            cmd_vel_msg.angular.z = 0.0;
        } else if (left_obstacle == 0 && front_obstacle == 0 && right_obstacle == 1) {
            // Move forward
            RCLCPP_INFO(this->get_logger(), "Moving Forward and Left");
            if(right<0.2){
                cmd_vel_msg.linear.x=0.0;
                cmd_vel_msg.angular.z=0.5;
            }
            else{
            cmd_vel_msg.linear.x = 0.2;
            cmd_vel_msg.angular.z = 0.05;
            }
        } else if (left_obstacle == 0 && front_obstacle == 1 && right_obstacle == 0) {
            // Turn right
            RCLCPP_INFO(this->get_logger(), "Moving Right");
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = -0.8;
        } else if (left_obstacle == 1 && front_obstacle == 0 && right_obstacle == 0) {
            // Move forward rightish
            RCLCPP_INFO(this->get_logger(), "Moving Forward and Right");
            if(left<0.2){
                cmd_vel_msg.linear.x=0.0;
                cmd_vel_msg.angular.z=-0.5;
            }
            else{
            cmd_vel_msg.linear.x = 0.2;
            cmd_vel_msg.angular.z = -0.05;
            }
        } else if (left_obstacle == 1 && front_obstacle == 0 && right_obstacle == 1) {
            // Still move
            RCLCPP_INFO(this->get_logger(), "Moving Forward");
            cmd_vel_msg.linear.x = 0.2;
            cmd_vel_msg.angular.z = 0.0;
        }
        else if (left_obstacle == 0 && front_obstacle == 1 && right_obstacle == 1) {
            // Turn left
            RCLCPP_INFO(this->get_logger(), "Turning Left");
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.8;
        } else if (left_obstacle == 1 && front_obstacle == 1 && right_obstacle == 0) {
            // Turn right
            RCLCPP_INFO(this->get_logger(), "Turning Right");
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = -0.8;
        } else if (left_obstacle == 1 && front_obstacle == 1 && right_obstacle == 1) {
            // Rotate 180 degrees
            RCLCPP_INFO(this->get_logger(), "Turning Back");
            cmd_vel_msg.linear.x = 0.0;
            if(cmd_vel_msg.angular.z >0){
                cmd_vel_msg.angular.z = 1.0;
            }
            else{
                cmd_vel_msg.angular.z = -1.0;
            }
            
        }

        cmd_vel_pub_->publish(cmd_vel_msg);
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Get yaw from quaternion (for simplicity in 2D)
    auto &orientation = msg->pose.pose.orientation;
    double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}
    double linear_speed_;
    double angular_speed_;
    double obstacle_distance_;
    double goal_x_, goal_y_;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    
    std::string robot_namespace_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WanderNode>();

    // Allow setting parameters from the command line
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
