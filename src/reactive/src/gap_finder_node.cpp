#include "gap_finder_node.hpp"
using std::vector;

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    // Declare with a default value
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width_extended", 0.55);
    this->declare_parameter("disparity_threshold", 1.5);
    this->declare_parameter("fov_half_angle_deg", 90.0);
    this->declare_parameter("minimum_gap_threshold", 0.1);

    // Read into member variables
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GapFinderNode::lidar_callback, this, std::placeholders::_1));

    gap_pub_ = this->create_publisher<reactive::msg::Gap>("gap", 10);
}

void GapFinderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    extend_obstacles(scan_msg, ranges);

    auto gap = find_furthest_gap(scan_msg, ranges);

    if (gap.first == -1)
    {
        return;
    }

    int furthest_point = find_furthest_point(ranges, gap);

    reactive::msg::Gap gap_msg;
    gap_msg.angles.reserve(gap.second - gap.first);
    gap_msg.ranges.reserve(gap.second - gap.first);

    for (size_t i = gap.first; i < static_cast<size_t>(gap.second); ++i)
    {
        gap_msg.angles.push_back(scan_msg->angle_min + scan_msg->angle_increment * i);
        gap_msg.ranges.push_back(ranges[i]);
    }

    gap_msg.target_angle = scan_msg->angle_min + scan_msg->angle_increment * furthest_point;
    gap_msg.target_range = ranges[furthest_point];

    gap_pub_->publish(gap_msg);
}

vector<float> GapFinderNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    // Capping values past max_lidar_range, and rejecting values past fov angle range
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        if (std::abs(scan_msg->angle_min + scan_msg->angle_increment * i) > fov_half_angle_)
        {
            ranges[i] = 0;
        }
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    return ranges;
}

void GapFinderNode::extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    if (ranges.empty()) return;

    const vector<float> original = ranges; // raw values for disparity comparisons

    float prev_reading = original[0];
    for (size_t i = 0; i < original.size(); ++i)
    {
        float current_reading = original[i];

        if (std::abs(prev_reading - current_reading) > disparity_threshold_)
        {
            float closer_range = std::min(prev_reading, current_reading);

            if (closer_range > 1e-3f) // avoid divide-by-zero / near-zero blowup
            {
                double theta = 2.0 * std::atan2(car_width_extended_ / 2.0, closer_range);
                theta = std::min(theta, M_PI); // also cap max bubble angle as a safety net

                size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);

                size_t start = (i > index_increment) ? i - index_increment : 0;
                size_t end = std::min(i + index_increment + 1, ranges.size());

                for (size_t j = start; j < end; ++j) {
                    ranges[j] = std::min(ranges[j], closer_range);
                }
            }
        }

        prev_reading = current_reading; // compare against raw values, not mutated output
    }
}

std::pair<int, int> GapFinderNode::find_furthest_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                                                     const vector<float> &ranges)
{
    int best_start = -1;
    int best_end = -1;
    float furthest_range = 0.0;

    size_t i = 0;
    while (i < ranges.size())
    {
        // Skip points that aren't part of a "free" gap
        if (ranges[i] <= minimum_gap_threshold_)
        {
            ++i;
            continue;
        }

        // Found the start of a gap - walk forward to find its end
        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_)
        {
            ++i;
        }
        size_t gap_end = i; // exclusive

        // Check if the gap is wide enough for the car to fit through,
        // using the closest range within the gap as the worst-case radius
        float min_range_in_gap = *std::min_element(ranges.begin() + gap_start, ranges.begin() + gap_end);
        double theta = 2.0 * std::atan2(car_width_extended_ / 2.0, min_range_in_gap);
        size_t min_index_width = static_cast<size_t>(theta / scan_msg->angle_increment);

        if ((gap_end - gap_start) < min_index_width)
        {
            continue; // gap too narrow for the car, skip it
        }

        // Gap is valid - check if it contains a further point than the best gap found so far
        for (size_t j = gap_start; j < gap_end; ++j)
        {
            if (ranges[j] > furthest_range)
            {
                furthest_range = ranges[j];
                best_start = static_cast<int>(gap_start);
                best_end = static_cast<int>(gap_end);
            }
        }
    }

    return {best_start, best_end};
}

int GapFinderNode::find_furthest_point(vector<float> &ranges, const std::pair<int, int> &gap) {
    if (gap.first == -1) return -1;

    int furthest = gap.first;
    for (int i = gap.first; i < gap.second; ++i) {
        if (ranges[i] > ranges[furthest])
        {
            furthest = i;
        }
    }

    return furthest;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFinderNode>());
    rclcpp::shutdown();
    return 0;
}
