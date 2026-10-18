#pragma once

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::vector<Waypoint> waypoints_;
    double lookahead_base_;        // base lookahead L0 (m)
    double lookahead_min_;         // lower clamp (m)
    double lookahead_max_;         // upper clamp (m)
    double lookahead_speed_gain_;  // extra lookahead (m) per m/s of speed
    size_t last_goal_index_ = 0;   // index of last chosen goal
    double wheel_base_;            // front-to-rear axle distance (m)
    bool initialized_ = false;     // false until first odom, then closest waypoint is found once

    /// @brief Loads the waypoint path from a file into waypoints_.
    /// @param path Path to the waypoint file (e.g. a CSV with x, y per row).
    void load_waypoints(const std::string& path);

    /// @brief Computes the speed-scaled lookahead distance.
    /// @param v Current forward speed of the car (m/s).
    /// @return Lookahead distance in meters.
    double currentLookahead(double v) const;

    /// @brief Odometry callback: finds the goal waypoint at the current lookahead
    ///        distance in front of the car, and computes the pure pursuit steering.
    /// @param msg Odometry message with the car's pose and velocity.
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
};
