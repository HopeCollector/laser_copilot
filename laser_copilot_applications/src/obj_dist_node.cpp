#include <rclcpp/rclcpp.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "common.hh"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
namespace laser_copilot_applications {
class obj_dist : public rclcpp::Node {
public:
  using point_t = pcl::PointXYZ;
  using cloud_t = pcl::PointCloud<point_t>;
  using objs_t = std::array<double, GROUP_NUM>;
  using cloud_ptr_t = std::shared_ptr<cloud_t>;
  using objs_ptr_t = std::shared_ptr<objs_t>;

  explicit obj_dist(const rclcpp::NodeOptions &options)
      : Node("obj_dist_node", options) {
    load_param();
    init_callback();
  }

private:

  void load_param() {
    z_max_ = declare_parameter("z_max", 0.5);
    z_min_ = declare_parameter("z_min", -0.5);
    distance_ = declare_parameter("distance", 0.2);
  }

  void init_callback() {
    using namespace std::placeholders;
    pub_ls_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "out/laser_scan", 5);
    sub_lvx_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "sub/lvx", rclcpp::SensorDataQoS(),
        std::bind(&obj_dist::cb_lvx, this, _1));
    sub_pc2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "sub/pc2", 5, std::bind(&obj_dist::cb_pc2, this, _1));
  }

  void cb_lvx(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
    static sensor_msgs::msg::LaserScan prv_ls;
    sensor_msgs::msg::LaserScan ls = new_laser_scan_msg(msg->header);
    ls.scan_time = msg->points.back().offset_time / 1e9f;
    ls.time_increment = ls.scan_time / msg->point_num;
    auto &range_min = ls.range_min;
    auto &range_max = ls.range_max;
    auto &ranges = ls.ranges;
    for (int i = 0; i < msg->point_num; i++) {
      const auto &p = msg->points[i];
      if (!is_in_detect_range(p))
        continue;
      auto &range = ranges[id(p.x, p.y)];
      auto tmp = range;
      range = std::min(range, dist(p.x, p.y));
      range_max = std::max(range, range_max);
      range_min = std::min(range, range_min);
    }
    if (range_max == 0.0 && prv_ls.range_max != 0.0) {
      prv_ls.header = ls.header;
      ls = prv_ls;
    } else if (range_max > 0.0) {
      auto &prv_ranges = prv_ls.ranges;
      range_max = 0.0;
      range_min = MAX_DIST;
      prv_ranges.resize(GROUP_NUM);
      for (int i = 0; i < GROUP_NUM; i++) {
        if (ranges[i] < MAX_DIST) {
          prv_ranges[i] = ranges[i];
        } else {
          ranges[i] = prv_ranges[i];
        }
        if (ranges[i] < MAX_DIST) {
          range_max = std::max(ranges[i], range_max);
          range_min = std::min(ranges[i], range_min);
        }
      }
      copy_laser_scan_msg_expect_array(ls, prv_ls);
    }
    pub_ls_->publish(ls);
  }

  void cb_pc2(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    sensor_msgs::msg::LaserScan ls = new_laser_scan_msg(msg->header);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    size_t point_num = msg->height * msg->width;
    auto &range_min = ls.range_min;
    auto &range_max = ls.range_max;
    auto &ranges = ls.ranges;
    for (int i = 0; i < point_num; ++i, ++iter_x, ++iter_y, ++iter_z) {
      point_t  p {*iter_x, *iter_y, *iter_z};
      if (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z)) continue;
      if (!is_in_detect_range(p)) continue;
      auto &range = ranges[id(p.x, p.y)];
      range = std::min(range, dist(p.x, p.y));
      range_max = std::max(range, range_max);
      range_min = std::min(range, range_min);
    }
    std::iota(ls.intensities.begin(), ls.intensities.end(), 0);
    pub_ls_->publish(ls);
  }

  sensor_msgs::msg::LaserScan new_laser_scan_msg(const std_msgs::msg::Header &header) {
    sensor_msgs::msg::LaserScan msg;
    msg.header = header;
    msg.header.frame_id = "lidar_link";
    msg.angle_increment = RAD_INC;
    msg.angle_min = 0;
    msg.angle_max = 2 * M_PI;
    msg.range_min = MAX_DIST;
    msg.range_max = 0.0f;
    msg.ranges.resize(GROUP_NUM);
    msg.intensities.resize(GROUP_NUM);
    std::fill(msg.ranges.begin(), msg.ranges.end(), MAX_DIST);
    return msg;
  }

  template <typename T>
  bool is_in_detect_range(T p) {
    return p.z <= z_max_ && p.z >= z_min_ && dist(p.x, p.y) >= distance_;
  }

  void copy_laser_scan_msg_expect_array(sensor_msgs::msg::LaserScan &src,
                                        sensor_msgs::msg::LaserScan &dst) {
    dst.header = src.header;
    dst.angle_min = src.angle_min;
    dst.angle_max = src.angle_max;
    dst.angle_increment = src.angle_increment;
    dst.time_increment = src.time_increment;
    dst.scan_time = src.scan_time;
    dst.range_min = src.range_min;
    dst.range_max = src.range_max;
  }

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lvx_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_ls_;
  double z_max_, z_min_;
  double distance_;
};
}; // namespace laser_copilot_applications_applications

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot_applications::obj_dist)