#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using namespace std::chrono_literals;


double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square_mover");
    
    // Publisher for velocity commands
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscriber to the /odom topic
    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odom_callback);

    // Set parameters
    double linear_speed = 0.1;
    double angular_speed = M_PI / 20;
    double square_length = 1.0;
    rclcpp::WallRate loop_rate(10ms);

    geometry_msgs::msg::Twist message;

    for (int j = 0; j < 4; j++)
    {
        // Save initial position and orientation
        double start_x = current_x;
        double start_y = current_y;
        double start_yaw = current_yaw;

        // Move forward until the robot has traveled the square side length
        while (rclcpp::ok() && std::hypot(current_x - start_x, current_y - start_y) < square_length)
        {
            message.linear.x = linear_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }

        // Stop forward motion
        message.linear.x = 0;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();

        // Rotate until the robot has turned 90 degrees
        while (rclcpp::ok() && std::fabs(current_yaw - start_yaw) < M_PI_2)
        {
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }

        // Stop rotation
        message.angular.z = 0;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}