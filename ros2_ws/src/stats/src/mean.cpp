#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "example_interfaces/msg/float64.hpp" 
#include <iostream>


double sum;
int msg_count=0.0;
double mean=0.0;
std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Float64>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{   
    sum += msg->data;
    msg_count++;
    example_interfaces::msg::Float64 out_msg;
    mean = sum/msg_count;
    out_msg.data = mean;
    publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{   
    sum = 0.0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mean");
    auto subscription = 
    node -> create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
 
    publisher = node -> create_publisher<example_interfaces::msg::Float64>("mean", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}