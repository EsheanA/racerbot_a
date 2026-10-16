#ifndef PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT_NODE_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

struct Waypoint
{
    double x;
    double y;
};

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

private:
    void load_waypoints(const std::string& path);
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

    std::vector<Waypoint> waypoints_;
    double lookahead_;
    size_t last_goal_index_ = 0;
    bool initialized_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

#endif  // PURE_PURSUIT_NODE_HPP_
