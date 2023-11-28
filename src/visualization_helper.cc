#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace laser_copilot {
class visualization_helper : public rclcpp::Node {
public:
  explicit visualization_helper(const rclcpp::NodeOptions &options)
      : Node("visualization_helper", options) {
    tf_static_pub_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    timer_1hz_ = create_wall_timer(1s, [this]() { this->callback_1hz(); });
  }

private:
  void callback_1hz() { pub_tf_static(); }

  void pub_tf_static() {
    static bool is_created_msg = false;
    static geometry_msgs::msg::TransformStamped msg;
    if (!is_created_msg) {
      msg.header.frame_id = "x500_lidar/link";
      msg.child_frame_id = "lidar_link";
      is_created_msg = true;
    }
    msg.header.stamp = get_clock()->now();
    tf_static_pub_->sendTransform(msg);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_pub_;
};
}; // namespace laser_copilot

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot::visualization_helper)