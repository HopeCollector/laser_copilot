#include <Eigen/Eigen>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
namespace laser_copilot {
enum class controller_state { disarmed, armed, takeoff, hold, ctrl };

struct pid_controller {
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;
  double last_err = 0.0;
  double all_err = 0.0;
  double operator()(double setpoint, double measure) {
    double err = setpoint - measure;
    all_err += err;
    double delta = err - last_err;
    double output = kp * err + ki * all_err + kd * delta;
    last_err = err;
    return output;
  }
  double operator()(double err) {
    all_err += err;
    double delta = err - last_err;
    double output = kp * err + ki * all_err + kd * delta;
    last_err = err;
    return output;
  }
};

struct pose_t {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d angle_vel = Eigen::Vector3d::Zero();
};

double quaternion_to_yaw(Eigen::Quaterniond q) {
  Eigen::Vector3d vec = q * Eigen::Vector3d::UnitX();
  return std::atan2(vec[1], vec[0]);
}

using namespace std::chrono_literals;
class safe_fly_controller : public rclcpp::Node {
public:
  explicit safe_fly_controller(const rclcpp::NodeOptions &options)
      : Node("safe_fly_controller", options),
        cur_state_(controller_state::disarmed) {
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
    timer_50hz_ =
        create_wall_timer(20ms, std::bind(&safe_fly_controller::cb_50hz, this));
    max_speed_ = declare_parameter("max_speed", 2.0);
    max_acc_ = declare_parameter("max_acc", 1.0);
    max_jerk_ = declare_parameter("max_jerk", 0.1);
    max_yaw_speed_ = declare_parameter("max_yaw_speed", 30.0) / 180.0 * M_PI;
    yaw_frd_ned_ = declare_parameter("yaw_flu_ned", 90.0) / 180.0 * M_PI;
    set_target({0, 0, declare_parameter("init_height", 3.0)},
               {0.999, 0, 0, 0.044});
    ctrl_msg_.position = false;
    ctrl_msg_.attitude = true;
    ctrl_msg_.velocity = true;
    ctrl_msg_.acceleration = true;
    ctrl_msg_.body_rate = true;
    ctrl_msg_.actuator = false;
  }

private:
  void cb_odometry(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    prv_pose_ = cur_pose_;
    cur_pose_.position << msg->position[0], msg->position[1], msg->position[2];
    cur_pose_.orientation =
        Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    cur_pose_.linear_vel << msg->velocity[0], msg->velocity[1],
        msg->velocity[2];
    cur_pose_.angle_vel << msg->angular_velocity[0], msg->angular_velocity[1],
        msg->angular_velocity[2];
  }

  void cb_status(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
    RCLCPP_INFO_STREAM(get_logger(), int(msg->arming_state));
  }

  void cb_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    set_target(
        {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z},
        {msg->pose.orientation.w, msg->pose.orientation.x,
         msg->pose.orientation.y, msg->pose.orientation.z});
    RCLCPP_INFO_STREAM(get_logger(), "going to: " << tgt_position_.transpose());
  }

  void cb_50hz() {
    ctrl_msg_.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    pub_mode_ctrl_->publish(ctrl_msg_);
    go_to_target(tgt_position_, tgt_orientation_);
  }

  void set_target(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
    static const Eigen::Quaterniond T_flu_frd{
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())};
    static const Eigen::Quaterniond T_frd_ned{
        Eigen::AngleAxisd(yaw_frd_ned_, Eigen::Vector3d::UnitZ())};
    static const Eigen::Quaterniond T_flu_ned = T_frd_ned * T_flu_frd;
    tgt_position_ = T_flu_ned * position;
    tgt_orientation_ = T_flu_ned * orientation * T_flu_ned.conjugate();
    RCLCPP_INFO_STREAM(get_logger(), "going to: " << tgt_position_.transpose() << ", " << tgt_orientation_);
  }

  void go_to_target(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
    Eigen::Vector3d direction = tgt_position_ - cur_pose_.position;
    double speed = std::min(max_speed_, vel_pid(direction.norm()));
    double acc = speed - cur_pose_.linear_vel.norm();
    if (acc > max_acc_) {
      speed = cur_pose_.linear_vel.norm() + max_acc_;
      acc = max_acc_;
    }
    direction.normalize();
    float vyaw = yaw_pid(quaternion_to_yaw(tgt_orientation_),
                         quaternion_to_yaw(cur_pose_.orientation));
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
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_cmd_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      pub_mode_ctrl_;
  rclcpp::TimerBase::SharedPtr timer_50hz_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  px4_msgs::msg::OffboardControlMode ctrl_msg_;
  double max_speed_;
  double max_acc_;
  double max_jerk_;
  double max_yaw_speed_;
  double yaw_frd_ned_;
  pose_t cur_pose_;
  pose_t prv_pose_;
  Eigen::Vector3d tgt_position_;
  Eigen::Quaterniond tgt_orientation_;
  controller_state cur_state_;
  pid_controller vel_pid;
  pid_controller yaw_pid;
};
}; // namespace laser_copilot

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot::safe_fly_controller)