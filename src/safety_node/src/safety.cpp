#include "safety.hpp"

SafetyNode::SafetyNode() : rclcpp::Node("safety_node") {
  this->deadman_held_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/safety/deadman", 10);
  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&SafetyNode::joy_callback, this, std::placeholders::_1));
  
  this->declare_parameter("deadman_button", 4);

  deadman_button_ = this->get_parameter("deadman_button").as_int();
}

void SafetyNode::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg) {
  std_msgs::msg::Bool msg;
   if (static_cast<int>(joy_msg->buttons.size()) > deadman_button_) {
        if (joy_msg->buttons[deadman_button_] != 0) {
          msg.data = true;
        } else {
          msg.data = false;
        }
    } else {
        msg.data = false;
    }

    this->deadman_held_pub_->publish(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}