#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/joy.hpp>

class SafetyNode : public rclcpp::Node {
public:
  SafetyNode();
  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_held_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  int deadman_button_;
  
  bool last_deadman_state;
};