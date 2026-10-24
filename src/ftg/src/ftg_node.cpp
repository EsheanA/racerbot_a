#include "ftg_node.hpp"
using std::vector;


FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node")
{
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

   // ROS2 parameters
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("fov_half_angle_deg", 90.0);
    this->declare_parameter("car_width", 0.4);
    this->declare_parameter("minimum_gap_threshold", 1.0);
    this->declare_parameter("enable_safety", true);
    this->declare_parameter("safety_bubble_scale", 0.5);
    this->declare_parameter("prev_angle_adjuster", 0.0);


    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    car_width_ = this->get_parameter("car_width").as_double();
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();
    enable_safety_ = this->get_parameter("enable_safety").as_bool();
    safety_bubble_scale_ = this->get_parameter("safety_bubble_scale").as_double();
    prev_angle_adjuster_ = this->get_parameter("prev_angle_adjuster").as_double();

    prev_angle_ = 0;

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
    safety_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/brake", 10,
        std::bind(&FollowTheGapNode::safety_brake_callback, this, std::placeholders::_1));
}

vector<float> FollowTheGapNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    for (size_t i = 0; i < ranges.size(); ++i) {
        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        if (std::abs(angle) > fov_half_angle_) ranges[i] = 0.0f;
        if (!std::isfinite(ranges[i]) || ranges[i] < scan_msg->range_min) ranges[i] = 0.0f;
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    vector<float> smooth = ranges;

    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        smooth[i] = (ranges[i-1] + ranges[i] + ranges[i+1]) / 3.0f;
    }

    ranges = smooth;

    return ranges;
}

void FollowTheGapNode::draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
     // Find closest obstacle (ignoring 0 ranges since 0 will already be avoided)
    bool found_closest = false;
    size_t closest_index;
    for (size_t i = 1; i < ranges.size(); ++i) {
        if (ranges[i] != 0 && !found_closest) {
            found_closest = true;
            closest_index = i;
        }

        if (found_closest && ranges[i] != 0 && ranges[i] < ranges[closest_index]) {
            closest_index = i;
        }
    }

    if (!found_closest) return; // handle case where all ranges are 0

    // Draw safety bubble by calculating theta for the arc made by r = closest point with s = car width

    // Adjust size of bubble based on the angle between the closest obstacle and the car's facing direction
    double direction_adjustment = std::max((1 - 2 * std::abs((int)(closest_index - ranges.size())) / ((int)ranges.size())), 0);

    // Adjust based on distance
    double distance_adjustment = std::min((int)(0.1 / ranges[closest_index]), 1);

    double theta = 2.0 * std::atan2(car_width_ * safety_bubble_scale_ * std::max(direction_adjustment, distance_adjustment), ranges[closest_index]);

    size_t index_increment = (theta / scan_msg->angle_increment) / 2; // Used on each side so divide by 2

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;

    size_t end = std::min(closest_index + index_increment, ranges.size());

    for (size_t i = start; i < end; ++i) {
        ranges[i] = 0;
    }
}

vector<std::pair<int, int>> FollowTheGapNode::find_all_gaps(const vector<float>& ranges)
{
    vector<std::pair<int, int>> gaps;
    size_t i = 0;

    float threshold_modifier = ranges[ranges.size() / 4];

    float gap_threshold = std::max((float)minimum_gap_threshold_,threshold_modifier);
    
    while (i < ranges.size()) {
        if (ranges[i] <= gap_threshold) { 
            ++i; 
            continue; 
        }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > gap_threshold) {
            ++i;
        }
        size_t gap_end = i;
        
        gaps.push_back({static_cast<int>(gap_start), static_cast<int>(gap_end)});
    }
    
    return gaps;
}

std::pair<int, int> FollowTheGapNode::pick_best_gap(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const vector<float>& ranges, 
    const std::vector<std::pair<int, int>>& gaps)
{
    std::pair<int, int> best_gap = {-1, -1};
    float best_score = -1.0f;

    for (const auto& gap : gaps) {
        int gap_start = gap.first;
        int gap_end = gap.second;

        // Calculate average depth of the gap
        float sum = 0.0f;
        for (int j = gap_start; j < gap_end; ++j) {
            sum += ranges[j];
        }
        float avg_depth = sum / (gap_end - gap_start);
        
        // Ensure the gap is wide enough for the car
        size_t min_width = static_cast<size_t>((car_width_ / avg_depth) / scan_msg->angle_increment);
        if (static_cast<size_t>(gap_end - gap_start) < min_width) continue;

        // Scoring based on width * depth (as suggested in the image)
        float score = (gap_end - gap_start) * avg_depth;

        if (score > best_score) {
            best_score = score;
            best_gap = gap;
        }
    }
    
    return best_gap;
}

int FollowTheGapNode::pick_best_point(const std::pair<int, int>& best_gap)
{
    if (best_gap.first == -1 || best_gap.second == -1) {
        return -1; // Invalid gap
    }
    
    // Pick the center of the gap (as suggested in the image)
    return best_gap.first + (best_gap.second - best_gap.first) / 2;
}

void FollowTheGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    int safety_publisher_count = this->safety_brake_sub_->get_publisher_count();
    if (enable_safety_ && (safety_brake_engaged_ || safety_publisher_count == 0)) {
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = 0.0;
        drive_msg_publisher->publish(drive_msg);
        return;
    }

    auto ranges = preprocess_lidar(scan_msg);
    draw_safety_bubble(scan_msg, ranges);
    auto gaps = find_all_gaps(ranges);
    auto best_gap = pick_best_gap(scan_msg, ranges, gaps);
    int best_idx = pick_best_point(best_gap);

    if (best_idx == -1)
    {
        RCLCPP_WARN(this->get_logger(), "No valid gap found");
        return;
    }

    float current_angle = scan_msg->angle_min + scan_msg->angle_increment * best_idx;

    // adjust angle based on previous
    float steering_angle = prev_angle_ * prev_angle_adjuster_ + current_angle * (1 - prev_angle_adjuster_);
    prev_angle_ = steering_angle;

    float abs_angle = std::abs(steering_angle);

    float speed;

    // --- Angle-based factor ---
    double max_angle = 30.0 * M_PI / 180.0; // angle at/beyond which speed hits its minimum
    double min_angle_speed = 0.5;
    double max_angle_speed = 4.0;

    double t_angle = std::clamp(abs_angle / max_angle, 0.0, 1.0);
    double angle_speed = max_angle_speed - t_angle * (max_angle_speed - min_angle_speed);

    // --- Range-based factor ---
    double target_range = static_cast<float>(ranges[best_idx]);
    double min_range = 0.5;  // range at/below which speed hits its minimum
    double max_range = 4.0;  // range at/above which speed hits its maximum
    double min_range_speed = 0.5;
    double max_range_speed = 4.0;

    double t_range = std::clamp((target_range - min_range) / (max_range - min_range), 0.0, 1.0);
    double range_speed = min_range_speed + t_range * (max_range_speed - min_range_speed);

    // --- Combine: take the more conservative (lower) of the two ---
    speed = static_cast<float>(std::min(angle_speed, range_speed) * 1.2);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = scan_msg->header.stamp;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    drive_msg_publisher->publish(drive_msg);
}

void FollowTheGapNode::safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg) {
    safety_brake_engaged_ = safety_brake_msg->data;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}
