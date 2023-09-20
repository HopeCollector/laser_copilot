#include <rclcpp/rclcpp.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
namespace laser_copilot {

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr int GROUP_NUM = 72;
constexpr float RAD_INC = 2 * M_PI / GROUP_NUM;
constexpr float MAX_DIST = 1000.0f;

inline int id(double x, double y) {
  return static_cast<int>((atan2(y, x) * RAD_TO_DEG + (y < 0 ? 360.0 : 0.0)) /
                          5) % GROUP_NUM;
}

inline float dist(double x, double y) { return std::sqrt(x * x + y * y); }

class obj_dist : public rclcpp::Node {
public:
  explicit obj_dist(const rclcpp::NodeOptions &options)
      : Node("obj_dist_node", options) {
    decl_param();
    load_param();
    is_changed_.resize(GROUP_NUM);
    pub_ls_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "out/laser_scan", rclcpp::SensorDataQoS());
    sub_lvx_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "sub/lvx", rclcpp::SensorDataQoS(),
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
          this->lvx_cb(msg);
        });
  }

private:
  void decl_param() {
    declare_parameter("z_max", 0.5);
    declare_parameter("z_min", -0.5);
    declare_parameter("distance", 0.2);
  }

  void load_param() {
    get_parameter("z_max", z_max_);
    get_parameter("z_min", z_min_);
    get_parameter("distance", distance_);
  }

  void lvx_cb(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
    static std::vector<float> prv_ranges;
    std::fill(is_changed_.begin(), is_changed_.end(), false);
    sensor_msgs::msg::LaserScan ls = new_laser_scan_msg(msg->header);
    ls.scan_time = msg->points.back().offset_time / 1e9f;
    auto &range_min = ls.range_min;
    auto &range_max = ls.range_max;
    auto &ranges = ls.ranges;
    for (int i = 0; i < msg->point_num; i++) {
      const auto &p = msg->points[i];
      if (!is_in_detect_range(p))
        continue;
      auto idx = id(p.x, p.y);
      auto &range = ranges[idx];
      auto tmp = range;
      range = std::min(range, dist(p.x, p.y));
      range_max = std::max(range, range_max);
      range_min = std::min(range, range_min);
      is_changed_[idx] = true;
    }
    if (!prv_ranges.empty()) {
      for (int i = 0; i < GROUP_NUM; i++) {

        if (is_changed_[i])
          continue;
        ranges[i] = prv_ranges[i];
      }
    }
    pub_ls_->publish(ls);
    prv_ranges.swap(ranges);
  }

  sensor_msgs::msg::LaserScan new_laser_scan_msg(const std_msgs::msg::Header &header) {
    sensor_msgs::msg::LaserScan msg;
    msg.header = header;
    msg.header.frame_id = "base_link";
    msg.angle_increment = RAD_INC;
    msg.angle_min = 0;
    msg.angle_max = 2 * M_PI;
    msg.range_min = MAX_DIST;
    msg.range_max = 0.0f;
    msg.ranges.resize(GROUP_NUM);
    std::fill(msg.ranges.begin(), msg.ranges.end(), MAX_DIST);
    return msg;
  }

  template <typename T>
  bool is_in_detect_range(T p) {
    return p.z <= z_max_ && p.z >= z_min_ && dist(p.x, p.y) >= distance_;
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lvx_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_ls_;
  std::vector<bool> is_changed_;
  double z_max_, z_min_;
  double distance_;
};
}; // namespace laser_copilot

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot::obj_dist)