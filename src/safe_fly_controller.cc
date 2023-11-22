#include "common.hh"
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
namespace laser_copilot {
using namespace std::chrono_literals;
class safe_fly_controller : public rclcpp::Node {
public:
  explicit safe_fly_controller(const rclcpp::NodeOptions &options)
      : Node("safe_fly_controller", options), timer_50hz_(nullptr) {
    init_pub_sub();
    load_param();
    init_ctrl_msg();
  }

private:
  void init_pub_sub() {
    using std::placeholders::_1;
    sub_position_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&safe_fly_controller::cb_odometry, this, _1));
    sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
        std::bind(&safe_fly_controller::cb_status, this, _1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "sub/goal", 5, std::bind(&safe_fly_controller::cb_goal, this, _1));
    pub_cmd_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
    pub_mode_ctrl_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
  }

  void load_param() {
    max_speed_ = declare_parameter("max_speed", 2.0);
    max_acc_ = declare_parameter("max_acc", 1.0);
    max_yaw_speed_ = declare_parameter("max_yaw_speed", 30.0) / 180.0 * M_PI;
    yaw_frd_ned_ = declare_parameter("yaw_flu_ned", 90.0) / 180.0 * M_PI;
  }

  void init_ctrl_msg() {
    ctrl_msg_.position = false;
    ctrl_msg_.attitude = true;
    ctrl_msg_.velocity = true;
    ctrl_msg_.acceleration = true;
    ctrl_msg_.body_rate = true;
    ctrl_msg_.actuator = false;
  }

  void cb_odometry(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    prv_pose_ = cur_pose_;
    cur_pose_.position << msg->position[0], msg->position[1], msg->position[2];
    cur_pose_.orientation =
        Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    cur_pose_.linear_vel << msg->velocity[0], msg->velocity[1],
        msg->velocity[2];
    cur_pose_.angle_vel << msg->angular_velocity[0], msg->angular_velocity[1],
        msg->angular_velocity[2];
    cur_pose_.yaw = quaternion_to_yaw(cur_pose_.orientation);
  }

  void cb_status(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
    RCLCPP_INFO_STREAM(get_logger(), int(msg->arming_state));
  }

  void cb_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    auto &frame_id = msg->header.frame_id;
    set_target(
        msg->header.frame_id.find("ned") == std::string::npos,
        {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z},
        {msg->pose.orientation.w, msg->pose.orientation.x,
         msg->pose.orientation.y, msg->pose.orientation.z});
    if (!timer_50hz_) {
      timer_50hz_ = create_wall_timer(
          20ms, std::bind(&safe_fly_controller::cb_50hz, this));
    }
  }

  void cb_50hz() {
    ctrl_msg_.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    pub_mode_ctrl_->publish(ctrl_msg_);
    go_to_target();
  }

  void set_target(bool is_need_transform, Eigen::Vector3d position,
                  Eigen::Quaterniond orientation) {
    static bool is_init_transformations = false;
    static Eigen::Affine3d T_flu_ned;
    if (!is_init_transformations) {
      Eigen::Affine3d T_flu_frd = Eigen::Affine3d::Identity();
      T_flu_frd.linear() =
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

      Eigen::Affine3d T_frd_ned = Eigen::Affine3d::Identity();
      T_frd_ned.linear() =
          Eigen::AngleAxisd(yaw_frd_ned_, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      T_flu_ned = T_frd_ned * T_flu_frd;
      is_init_transformations = true;
    }
    if (is_need_transform) {
      Eigen::Affine3d T_sp_flu = Eigen::Affine3d::Identity();
      T_sp_flu.translation() = position;
      T_sp_flu.linear() = orientation.toRotationMatrix();
      T_sp_flu = T_flu_ned * T_sp_flu;
      target_ = setpoint_t(Eigen::Vector3d{T_sp_flu.translation()},
                           Eigen::Quaterniond{T_sp_flu.linear()});
    } else {
      target_ = setpoint_t(position, orientation);
    }
    RCLCPP_INFO_STREAM(get_logger(), "going to: " << target_);
  }

  void go_to_target() {
    Eigen::Vector3d direction = target_.position - cur_pose_.position;
    double speed = std::min(max_speed_, vel_pid_(direction.norm()));
    double acc = speed - cur_pose_.linear_vel.norm();
    if (acc > max_acc_) {
      speed = cur_pose_.linear_vel.norm() + max_acc_;
      acc = max_acc_;
    }
    direction.normalize();
    float vyaw =
        yaw_pid_(target_.yaw, quaternion_to_yaw(cur_pose_.orientation));
    if (cur_pose_.position[2] >= -1.0) {
      vyaw = 0.0;
    } else if (std::abs(vyaw) > max_yaw_speed_) {
      vyaw = vyaw / std::abs(vyaw) * max_yaw_speed_;
    }
    action_set_speed((direction * speed).cast<float>(),
                     (direction * acc).cast<float>(), vyaw);
  }

  void action_set_speed(Eigen::Vector3f vel, Eigen::Vector3f acc, float vyaw) {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    msg.position = {NAN, NAN, NAN};
    msg.yaw = NAN;
    msg.velocity = {vel[0], vel[1], vel[2]};
    msg.acceleration = {acc[0], acc[1], acc[2]};
    msg.jerk = {NAN, NAN, NAN};
    msg.yawspeed = vyaw;
    pub_cmd_->publish(msg);
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      sub_position_ = nullptr;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_ =
      nullptr;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_ =
      nullptr;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_cmd_ =
      nullptr;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      pub_mode_ctrl_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_50hz_ = nullptr;
  px4_msgs::msg::OffboardControlMode ctrl_msg_;
  double max_speed_;
  double max_acc_;
  double max_yaw_speed_;
  double yaw_frd_ned_;
  pose_t cur_pose_;
  pose_t prv_pose_;
  setpoint_t target_;
  pid_controller vel_pid_;
  pid_controller yaw_pid_;
};
}; // namespace laser_copilot

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot::safe_fly_controller)