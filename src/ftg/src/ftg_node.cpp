#include "ftg_node.hpp"
using std::vector;


FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node")
{
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

   // ROS2 parameters
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("fov_half_angle_deg", 90.0);
    this->declare_parameter("car_width", 0.35);
    this->declare_parameter("minimum_gap_threshold", 1.0);

    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    car_width_ = this->get_parameter("car_width").as_double();
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
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

    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        ranges[i] = (ranges[i-1] + ranges[i] + ranges[i+1]) / 3.0f;
    }

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

    // Draw safety buble by calculating theta for the arc made by r = closest point with s = car width

    double theta = 2.0 * std::atan2(car_width_ / 2.0, ranges[closest_index]);

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
    
    while (i < ranges.size()) {
        if (ranges[i] <= minimum_gap_threshold_) { 
            ++i; 
            continue; 
        }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_) {
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
    auto ranges = preprocess_lidar(scan_msg);
    draw_safety_bubble(scan_msg, ranges);
    auto gaps = find_all_gaps(ranges);
    auto best_gap = pick_best_gap(scan_msg, ranges, gaps);
    int best_idx = pick_best_point(best_gap);

    if (best_idx == -1)
    {
        RCLCPP_WARN(this->get_logger(), "No valid gap found");
        // Braking logic: if no gap, gently brake to a stop instead of instantly dying
        current_speed_ = std::max(0.0f, current_speed_ - 0.2f); 
        return;
    }

    float steering_angle = scan_msg->angle_min + scan_msg->angle_increment * best_idx;
    float abs_angle = std::abs(steering_angle);

    // New speed stuff below this line (everything above is untouched)
    float min_speed = 2.0f; // change max/min here
    float max_speed = 5.0f;

    // Addition 1: checking walls next to car
    int center_idx = ranges.size() / 2; 
    int idx_90_deg = (M_PI / 2.0) / scan_msg->angle_increment; // Number of LIDAR scans for 90 degrees. 
    int left_idx = center_idx + idx_90_deg; // LIDAR index for 90 degrees left
    int right_idx = center_idx - idx_90_deg; // LIDAR index for 90 degrees right 
    
    float left_dist = (left_idx < (static_cast<int>(scan_msg->ranges.size()))) ? scan_msg->ranges[left_idx] : max_lidar_range_; // checks distance for left_idx IF it is in the lidar array at the current moment.
    float right_dist = (right_idx >= 0) ? scan_msg->ranges[right_idx] : max_lidar_range_; // checks distance for right_idx IF it is in the lidar array at the current moment.
    


    float speed_lateral = max_speed; // Set the default speed for car between walls as max speed.1
    if (left_dist < (car_width_ / 1)  || right_dist < (car_width_ / 1) ) { // If the wall to the left or right side of the car is less than the car's width, slow down. (Can change this by changing (car_width_ / 1))
        speed_lateral = min_speed; // Possibly make this a smoother function to dynamically change depending on distance to parallel walls?
    }

    // Addition 2: check if distnace in front of car is small, if so, slow down
    float target_distance = ranges[best_idx];
    float speed_forward = min_speed + (target_distance / max_lidar_range_) * (max_speed - min_speed); 
    
    // Addition 3: reduce speed depending on angle being turned. (NOT a piecewise funciton anymore, uses cos(x) to smooth out velocity changes.)
   float speed_steer = max_speed; // set default speed to max if car is not turning by atleast 10 degrees
    if (abs_angle >= (10 * (M_PI/180))){ // if turn requires more than 10 degrees (in radians), move speed to this speed.
    speed_steer = 1.0 + (min_speed) * std::cos(abs_angle);
    }

    // Choosing the minimum speed out of all the different speed's depending on car's position. (also include max_speed in case of car is in a location with no block ahead, no turns or close walls.)
    float target_speed = std::min({speed_lateral, speed_forward, speed_steer, max_speed});
    
    // Addition 5: acceleration system to smooth out car velocity (using another variable to increment up to the chosen target speed allows us to not have sudden jumps in speed.)
    float max_accel_step = 0.1f; // change this to change acceleration.
    if (target_speed > current_speed_ + max_accel_step) current_speed_ += max_accel_step;
    else if (target_speed < current_speed_ - max_accel_step) current_speed_ -= max_accel_step;
    else current_speed_ = target_speed;
    
    // ------------------------

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = scan_msg->header.stamp;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = current_speed_; // publish smoothed out velocity
    drive_msg_publisher->publish(drive_msg);

    // print the current speed and wall distances at most once every second (just for checking if all speeds are working.)
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 
        1000, 
        "Speed: %.2f | Left : %.2f | Right : %.2f", 
        current_speed_, left_dist, right_dist
    );


}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}
