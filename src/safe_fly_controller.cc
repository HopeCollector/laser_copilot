#include "common.hh"
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
namespace laser_copilot {
using namespace std::chrono_literals;
class safe_fly_controller : public rclcpp::Node {
public:
  explicit safe_fly_controller(const rclcpp::NodeOptions &options)
      : Node("safe_fly_controller", options){
    init_cb();
    load_param();
    init_ctrl_msg();
  }

private:
  void init_cb() {
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
    pub_dbg_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/pub/debug", 5);
    timer_once_1s_ = create_wall_timer(1s, [this](){
      this->timer_once_1s_->cancel();
      this->T_odomflu_odomned_.linear() = Eigen::Matrix3d(
          Eigen::AngleAxisd(this->cur_pose_.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      this->T_odomned_odomflu_ = this->T_odomflu_odomned_.inverse();
      this->T_localflu_localfrd_.linear() =
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
      this->T_localfrd_localflu_ = this->T_localflu_localfrd_.inverse();
      RCLCPP_INFO_STREAM(get_logger(),
                         "detect init yaw(degree) in odom_ned frame: "
                             << this->cur_pose_.yaw / M_PI * 180.0);
    });
  }

  void load_param() {
    max_speed_ = declare_parameter("max_speed", 2.0);
    max_acc_ = declare_parameter("max_acc", 1.0);
    max_jerk_ = declare_parameter("max_jerk", 0.2);
    max_yaw_speed_ = declare_parameter("max_yaw_speed", 30.0) / 180.0 * M_PI;
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
    Eigen::Affine3d T_localfrd_odomned = Eigen::Affine3d::Identity();
    T_localfrd_odomned.translation() << msg->position[0], msg->position[1],
        msg->position[2];
    T_localfrd_odomned.linear() =
        Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3])
            .toRotationMatrix();
    Eigen::Affine3d T_localflu_odomflu = T_odomned_odomflu_ * T_localfrd_odomned * T_localflu_localfrd_;
    cur_pose_.position = T_localflu_odomflu.translation();
    cur_pose_.orientation = T_localflu_odomflu.linear();
    cur_pose_.linear_vel << msg->velocity[0], msg->velocity[1],
        msg->velocity[2];
    cur_pose_.angle_vel << msg->angular_velocity[0], msg->angular_velocity[1],
        msg->angular_velocity[2];
    cur_pose_.linear_vel = T_odomned_odomflu_ * cur_pose_.linear_vel;
    cur_pose_.angle_vel = T_odomned_odomflu_ * cur_pose_.angle_vel;
    cur_pose_.yaw = quaternion_to_yaw(cur_pose_.orientation);
    cur_pose_.stamp = msg->timestamp * 1000; // us -> ns
  }

  void cb_status(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
    RCLCPP_INFO_STREAM(get_logger(), int(msg->arming_state));
  }

  void cb_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    target_ = setpoint_t(
        {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z},
        {msg->pose.orientation.w, msg->pose.orientation.x,
         msg->pose.orientation.y, msg->pose.orientation.z});
    if(!timer_50hz_) {
      timer_50hz_ = this->create_wall_timer(
          20ms, std::bind(&safe_fly_controller::cb_50hz, this));
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "goint to: " << target_.position.transpose() << " "
                                    << target_.yaw / M_PI * 180.0);
  }

  void cb_50hz() {
    ctrl_msg_.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    pub_mode_ctrl_->publish(ctrl_msg_);
    go_to_target();
  }

  void go_to_target() {
    Eigen::Vector3d speed_vec = target_.position - cur_pose_.position;
    Eigen::Vector3d speed_dir = speed_vec / speed_vec.norm();
    speed_vec = vel_pid_(speed_vec.norm()) * speed_dir;
    Eigen::Vector3d acc_vec = speed_vec - cur_pose_.linear_vel;
    Eigen::Vector3d acc_dir = acc_vec / acc_vec.norm();
    acc_vec = std::min(max_acc_, acc_vec.norm()) * acc_dir;
    speed_vec = std::min(max_speed_, (cur_pose_.linear_vel + acc_vec).norm()) *
                speed_dir;
    acc_vec = speed_vec - cur_pose_.linear_vel;

    double vyaw = target_.yaw - cur_pose_.yaw;
    if (vyaw > M_PI) {
      vyaw -= 2 * M_PI;
    } else if (vyaw < -M_PI) {
      vyaw += 2 * M_PI;
    }
    int vyaw_dir = vyaw / std::abs(vyaw);
    vyaw = std::min(std::abs(vyaw), max_yaw_speed_) * vyaw_dir;

    action_set_speed(speed_vec, acc_vec, vyaw);

    std_msgs::msg::Float64MultiArray dbg_msg;
    dbg_msg.data.push_back(cur_pose_.position[0]);
    dbg_msg.data.push_back(cur_pose_.position[1]);
    dbg_msg.data.push_back(cur_pose_.position[2]);
    dbg_msg.data.push_back(speed_vec.norm());
    dbg_msg.data.push_back(acc_vec.norm());
    dbg_msg.data.push_back(vyaw); // 5
    dbg_msg.data.push_back(cur_pose_.linear_vel.norm()); // 6
    dbg_msg.data.push_back(cur_pose_.yaw / M_PI * 180.0); // 7
    pub_dbg_->publish(dbg_msg);
  }

  void action_set_speed(Eigen::Vector3d vel, Eigen::Vector3d acc, double vyaw) {
    vel = T_odomflu_odomned_ * vel;
    acc = T_odomflu_odomned_ * acc;
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    msg.position = {NAN, NAN, NAN};
    msg.yaw = NAN;
    msg.velocity = {float(vel[0]), float(vel[1]), float(vel[2])};
    msg.acceleration = {float(acc[0]), float(acc[1]), float(acc[2])};
    msg.jerk = {NAN, NAN, NAN};
    msg.yawspeed = (T_odomflu_odomned_ * Eigen::Vector3d{0, 0, vyaw})[2];
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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_dbg_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_50hz_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_once_1s_ = nullptr;
  px4_msgs::msg::OffboardControlMode ctrl_msg_;
  Eigen::Affine3d T_odomflu_odomned_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_odomned_odomflu_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localflu_localfrd_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localfrd_localflu_ = Eigen::Affine3d::Identity();
  double max_speed_;
  double max_acc_;
  double max_jerk_;
  double max_yaw_speed_;
  pose_t cur_pose_;
  pose_t prv_pose_;
  setpoint_t target_;
  pid_controller vel_pid_;
  pid_controller yaw_pid_;
};
}; // namespace laser_copilot

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot::safe_fly_controller)