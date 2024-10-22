#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "example_interfaces/msg/float64.hpp" 
#include <vector>
#include <algorithm>
#include <iostream>

std::vector<int> values;  // Vector to store incoming data
std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Float64>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{   
    // Add the new value to the vector
    values.push_back(msg->data);

    // Display the input values in the vector
    std::cout << "Input values: ";
    for (const int& val : values) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    // Sort the vector
    std::sort(values.begin(), values.end());

    // Display the sorted vector
    std::cout << "Sorted values: ";
    for (const int& val : values) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    // Compute the median
    double median;
    size_t size = values.size();
    if (size % 2 == 0) {
        median = (values[size / 2 - 1] + values[size / 2]) / 2.0;
    } else {
        median = values[size / 2];
    }

    // Publish the median
    example_interfaces::msg::Float64 out_msg;
    out_msg.data = median;
    publisher->publish(out_msg);

    // Display the computed median
    std::cout << "Published median: " << median << std::endl;
}

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("median");
    
    auto subscription = 
    node -> create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
 
    publisher = node -> create_publisher<example_interfaces::msg::Float64>("median", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
