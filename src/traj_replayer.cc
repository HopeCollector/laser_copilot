#include "common.hh"
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
namespace laser_copilot {
class traj_replayer : public rclcpp::Node {
public:
  explicit traj_replayer(const rclcpp::NodeOptions &options)
      : Node("traj_replayer", options) {
    using namespace std::placeholders;
    sub_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&traj_replayer::cb_vehicle_odom, this, _1));
    pub_sp_ =
        create_publisher<geometry_msgs::msg::PointStamped>("pub/setpoint", 5);
    yaw_frd_ned_ = declare_parameter("yaw_frd_nde", 90.0) / 180.0 * M_PI;
    file_path_ = declare_parameter("file_path", std::string(""));
    using namespace std::chrono_literals;
    timer_10hz_ =
        create_wall_timer(100ms, std::bind(&traj_replayer::cb_10hz, this));
  }

private:
  void cb_vehicle_odom(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {}

  void cb_10hz() {
    static const Eigen::Quaterniond T_flu_frd{
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())};
    static const Eigen::Quaterniond T_frd_ned{
        Eigen::AngleAxisd(yaw_frd_ned_, Eigen::Vector3d::UnitZ())};
    static const Eigen::Quaterniond T_flu_ned = T_frd_ned * T_flu_frd;
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_sp_;
  rclcpp::TimerBase::SharedPtr timer_10hz_;
  double yaw_frd_ned_;
  std::string file_path_;
  // std::vector<
};
}; // namespace laser_copilot

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot::traj_replayer)