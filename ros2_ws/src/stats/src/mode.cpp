#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "example_interfaces/msg/float64.hpp" 
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>

std::vector<int> values;  // Vector to store incoming data
std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Float64>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    // Add the new value to the vector
    values.push_back(msg->data);

    // Count the occurrences of each number
    std::map<int, int> frequency_map;
    for (const int& val : values) {
        frequency_map[val]++;
    }

    // Display the occurrences of each number
    std::cout << "Number occurrences:" << std::endl;
    for (const auto& pair : frequency_map) {
        std::cout << "Number: " << pair.first << " Count: " << pair.second << std::endl;
    }

    // Find the maximum occurrence count
    int max_count = 0;
    for (const auto& pair : frequency_map) {
        if (pair.second > max_count) {
            max_count = pair.second;
        }
    }

    // Find all numbers with the maximum occurrence count
    std::vector<int> modes;
    for (const auto& pair : frequency_map) {
        if (pair.second == max_count) {
            modes.push_back(pair.first);
        }
    }

    // Display the mode(s)
    std::cout << "Mode(s) with the highest occurrence: ";
    for (const int& mode : modes) {
        std::cout << mode << " ";
    }
    std::cout << std::endl;

    // Publish each mode (number with the highest occurrence)
    for (const int& mode : modes) {
        example_interfaces::msg::Float64 out_msg;
        out_msg.data = mode;
        publisher->publish(out_msg);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mode");

    auto subscription = 
    node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);

    publisher = node->create_publisher<example_interfaces::msg::Float64>("mode", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
