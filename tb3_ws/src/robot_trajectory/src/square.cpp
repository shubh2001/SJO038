#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])

{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);

    // while (rclcpp::ok())
    // {   
    //     message.linear.x = 1.0;
    //     message.angular.z = 1.0;
    //     publisher->publish(message);
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    // }

    int i=0, n=1000;
    while (rclcpp::ok() && i<n)
    {
        i++;
        message.linear.x = 0.1;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;

    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = 0;

    publisher->publish(message);

    i=0, n=1000;
    while (rclcpp::ok() && i<n)
    {
        i++;
        message.angular.z = (9.0 * 3.14)/180;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;

    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = 0;

    publisher->publish(message);

    rclcpp::shutdown();
    return 0;
}