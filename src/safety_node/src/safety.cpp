#include "safety.hpp"

SafetyNode::SafetyNode() : rclcpp::Node("safety_node") {
  this->deadman_held_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/safety/deadman", 10);
  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&SafetyNode::joy_callback, this, std::placeholders::_1));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}