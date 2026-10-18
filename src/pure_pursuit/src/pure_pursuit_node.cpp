#include "pure_pursuit_node.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    // ros2 parameters
    const auto track = declare_parameter<std::string>("track", "levine");
    lookahead_base_ = declare_parameter<double>("lookahead_base", 1.0);
    lookahead_min_ = declare_parameter<double>("lookahead_min", 0.8);
    lookahead_max_ = declare_parameter<double>("lookahead_max", 3.0);
    lookahead_speed_gain_ = declare_parameter<double>("lookahead_speed_gain", 0.3);
    wheel_base_ = declare_parameter<double>("wheelbase", 0.33);

    // loading waypoints from the package share directory
    const std::string path =
        ament_index_cpp::get_package_share_directory("pure_pursuit") +
        "/waypoints/" + track + ".csv";
    load_waypoints(path);

    // localization
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10,
        std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints for %s", waypoints_.size(), track.c_str());
}

void PurePursuit::load_waypoints(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open waypoint file: " + path);
    }

    std::string line;
    while (std::getline(file, line)) {
        // ignore empty lines or any starting wth # (comments)
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string x_str, y_str;
        if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',')) {
            continue;
        }

        try {
            Waypoint wp{std::stod(x_str), std::stod(y_str)};
            if (std::isfinite(wp.x) && std::isfinite(wp.y)) {
                waypoints_.push_back(wp);
            }
        } catch (const std::exception&) {
            // ignore rows that arent numbers
        }
    }

    if (waypoints_.empty()) {
        throw std::runtime_error("No valid waypoints found");
    }
}

double PurePursuit::currentLookahead(double v) const
{
    const double L = lookahead_base_ + lookahead_speed_gain_ * std::abs(v);
    return std::clamp(L, lookahead_min_, lookahead_max_);
}

void PurePursuit::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // current position and orientation of car
    double car_x = msg->pose.pose.position.x;
    double car_y = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // forward speed and speed-scaled lookahead
    const double speed = msg->twist.twist.linear.x;
    const double L = currentLookahead(speed);

    // First odom msg finds the closest waypoint to give the last_goal_index_ to start finding the waypoint from
    if (!initialized_) { // TODO: Handle when to reset
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < waypoints_.size(); i++) {
            double dx = waypoints_[i].x - car_x;
            double dy = waypoints_[i].y - car_y;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < best_dist) {
                best_dist = d;
                last_goal_index_ = i;
            }
        }
        initialized_ = true;
    }

    // searching forward through the waypoints
    for (size_t step = 0; step < waypoints_.size(); ++step) {
        size_t i = (last_goal_index_ + step) % waypoints_.size();
        double dx = waypoints_[i].x - car_x;
        double dy = waypoints_[i].y - car_y;

        // transforming world point to the car frame
        double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy; // how far ahead of car
        double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy; // how far to the left (positive) or right (negative)
        double distance = std::sqrt(dx * dx + dy * dy);

        // waypoint must be INFRONT of the car and ATLEAST L away.
        if (local_x > 0.0 && distance >= L) { // TODO: Interpolate between 2 points possibly
            last_goal_index_ = i;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Goal %zu: world(%.2f, %.2f) local(%.2f, %.2f)",
                i, waypoints_[i].x, waypoints_[i].y, local_x, local_y);

            /*
             * Add curvature steering and speed calculations here!!
             * (use local_x, local_y)
             *
             * 
             * 
             * 
             * 
             */
            return;
        }
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No suitable forward waypoint found"); // TODO: Handle failure
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
