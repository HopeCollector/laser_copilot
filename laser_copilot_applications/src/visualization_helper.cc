#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include "common.hh"

namespace laser_copilot_applications {
class visualization_helper : public rclcpp::Node {
public:
  explicit visualization_helper(const rclcpp::NodeOptions &options)
      : Node("visualization_helper", options) {
    tf_static_pub_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    timer_1hz_ = create_wall_timer(1s, [this]() { this->callback_1hz(); });
    sub_controller_info_ =
        create_subscription<std_msgs::msg::Float64MultiArray>(
            "sub/debug", 10,
            std::bind(&visualization_helper::cb_controller_info, this,
                      std::placeholders::_1));
    sub_gui_setpoint_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "sub/gui_setpoint", 10,
        std::bind(&visualization_helper::cb_gui_setpoint, this,
                  std::placeholders::_1));
    pub_path_stright_ =
        create_publisher<nav_msgs::msg::Path>("pub/path_stright", 10);
    pub_path_real_ = create_publisher<nav_msgs::msg::Path>("pub/path_real", 10);
    pub_setpoint_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "pub/setpoint", 10);
    pub_velocity_ = create_publisher<visualization_msgs::msg::Marker>(
        "pub/velocity", 10);

    msg_path_.header.frame_id = "odom";
  }

private:
  void cb_controller_info(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = get_clock()->now();
    marker.ns = "laser_copilot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points.resize(2);
    auto &start_point = marker.points[0];
    auto &end_point = marker.points[1];
    double speed = msg->data[3];
    start_point.x = msg->data[0];
    start_point.y = msg->data[1];
    start_point.z = msg->data[2];
    end_point.x = start_point.x + msg->data[8] * speed;
    end_point.y = start_point.y + msg->data[9] * speed;
    end_point.z = start_point.z + msg->data[10] * speed;
    marker.scale.x = 0.1 * speed;
    marker.scale.y = 0.2 * speed;
    marker.scale.z = 0.2 * speed;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    pub_velocity_->publish(marker);

    cur_pos_.position << msg->data[0], msg->data[1], msg->data[2];
    geometry_msgs::msg::PoseStamped msg_pose;
    msg_path_.header.stamp = get_clock()->now();
    msg_pose.header = msg_path_.header;
    msg_pose.pose.position.x = cur_pos_.position.x();
    msg_pose.pose.position.y = cur_pos_.position.y();
    msg_pose.pose.position.z = cur_pos_.position.z();
    msg_pose.pose.orientation.w = 1.0;
    msg_pose.pose.orientation.x = 0.0;
    msg_pose.pose.orientation.y = 0.0;
    msg_pose.pose.orientation.z = 0.0;
    msg_path_.poses.push_back(msg_pose);
    pub_path_real_->publish(msg_path_);
  }

  void cb_gui_setpoint(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto new_msg = *msg;
    new_msg.pose.position.z = cur_pos_.position.z();
    pub_setpoint_->publish(new_msg);

    nav_msgs::msg::Path msg_path_stright;
    msg_path_stright.header.frame_id = "odom";
    msg_path_stright.header.stamp = get_clock()->now();
    msg_path_stright.poses.resize(2);
    auto &start_point = msg_path_stright.poses[0];
    auto &end_point = msg_path_stright.poses[1];
    start_point.header = msg_path_stright.header;
    start_point.pose.position.x = cur_pos_.position.x();
    start_point.pose.position.y = cur_pos_.position.y();
    start_point.pose.position.z = cur_pos_.position.z();
    start_point.pose.orientation.w = 1.0;
    start_point.pose.orientation.x = 0.0;
    start_point.pose.orientation.y = 0.0;
    start_point.pose.orientation.z = 0.0;
    end_point.header = msg_path_stright.header;
    end_point.pose.position.x = new_msg.pose.position.x;
    end_point.pose.position.y = new_msg.pose.position.y;
    end_point.pose.position.z = new_msg.pose.position.z;
    end_point.pose.orientation.w = 1.0;
    end_point.pose.orientation.x = 0.0;
    end_point.pose.orientation.y = 0.0;
    end_point.pose.orientation.z = 0.0;
    pub_path_stright_->publish(msg_path_stright);

    msg_path_.poses.clear();
  }

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
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_controller_info_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gui_setpoint_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_stright_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_real_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_velocity_;
  pose_t cur_pos_;
  nav_msgs::msg::Path msg_path_;
};
}; // namespace laser_copilot_applications

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot_applications::visualization_helper)