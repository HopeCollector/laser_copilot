#include "common.hh"
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <queue>
#include <tf2_ros/static_transform_broadcaster.h>
namespace laser_copilot_applications {
class traj_replayer : public rclcpp::Node {
public:
  explicit traj_replayer(const rclcpp::NodeOptions &options)
      : Node("traj_replayer", options) {
    load_param();
    regist_callback();
  }

private:
  void load_param() {
    T_localflu_localfrd_.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    T_localfrd_localflu_ = T_localflu_localfrd_.inverse();
    load_setpoints(declare_parameter("file_path", std::string("")),
      declare_parameter("topic", std::string("")));
  }

  void regist_callback() {
    using namespace std::placeholders;
    sub_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&traj_replayer::cb_vehicle_odom, this, _1));
    pub_sp_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("pub/setpoint", 5);
    pub_path_ = 
        create_publisher<nav_msgs::msg::Path>("pub/path", 5);

    using namespace std::chrono_literals;
    timer_100hz_ =
        create_wall_timer(10ms, std::bind(&traj_replayer::cb_100hz, this));
    timer_1hz_ = create_wall_timer(1s, std::bind(&traj_replayer::cb_1hz, this));
    
  }

  void load_setpoints(std::string file_path, std::string topic) {
    rosbag2_cpp::Reader reader;
    rclcpp::Serialization<px4_msgs::msg::VehicleOdometry> serialization;
    px4_msgs::msg::VehicleOdometry px4_msg;
    Eigen::Affine3d T_localflu_odomflu = Eigen::Affine3d::Identity();
    reader.open(file_path);
    double yaw_frd_ned = NAN;
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (msg->topic_name != topic)
        continue;
      rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};
      serialization.deserialize_message(&serialized_msg, &px4_msg);
      if (std::isnan(yaw_frd_ned)) {
        yaw_frd_ned = quaternion_to_yaw(Eigen::Quaterniond{
            px4_msg.q[0], px4_msg.q[1], px4_msg.q[2], px4_msg.q[3]});
        T_odomflu_odomned_ = create_affine_flu_to_ned(yaw_frd_ned);
        T_odomned_odomflu_ = T_odomflu_odomned_.inverse();
      }
      T_localflu_odomflu.translation() << px4_msg.position[0],
          px4_msg.position[1], px4_msg.position[2];
      T_localflu_odomflu.linear() =
          Eigen::Quaterniond{px4_msg.q[0], px4_msg.q[1], px4_msg.q[2],
                             px4_msg.q[3]}
              .toRotationMatrix();
      T_localflu_odomflu =
          T_odomned_odomflu_ * T_localflu_odomflu * T_localfrd_localflu_;
      setpoints_.emplace_back(T_localflu_odomflu);
    }
    RCLCPP_INFO_STREAM(get_logger(), "load: " << setpoints_.size()
                                              << " points from " << file_path);
  }

  void cb_vehicle_odom(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    Eigen::Affine3d T_localfrd_odomned = Eigen::Affine3d::Identity();
    T_localfrd_odomned.translation() << msg->position[0], msg->position[1],
        msg->position[2];
    T_localfrd_odomned.linear() =
        Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3])
            .toRotationMatrix();
    Eigen::Affine3d T_localflu_odomflu =
        T_odomned_odomflu_ * T_localfrd_odomned * T_localflu_localfrd_;
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

  void cb_1hz(){
    static bool is_init_path_msg = false;
    static nav_msgs::msg::Path msg;
    if(!is_init_path_msg) {
      is_init_path_msg = true;
      msg.header.frame_id = "odom";
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      for (const auto& sp : setpoints_) {
        pose.pose.position.x = sp.position[0];
        pose.pose.position.y = sp.position[1];
        pose.pose.position.z = sp.position[2];
        Eigen::Quaterniond q(Eigen::AngleAxisd(sp.yaw, Eigen::Vector3d::UnitZ())
                                 .toRotationMatrix());
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        msg.poses.push_back(pose);
      }
    }
    msg.header.stamp = get_clock()->now();
    pub_path_->publish(msg);
  }

  void cb_100hz() {
    if (is_need_update_sp()) setpoints_.pop_front();
    auto sp = setpoints_.front().to_affine();

    Eigen::Vector3d position {sp.translation()};
    Eigen::Quaterniond orientation{sp.linear()};
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = get_clock()->now();
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    pub_sp_->publish(msg);
  }

  bool is_need_update_sp() {
    bool is_ok = false;
    is_ok = (cur_pose_.position - setpoints_.front().position).norm() < 3;
    return is_ok && setpoints_.size() > 1;
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_sp_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  rclcpp::TimerBase::SharedPtr timer_100hz_;
  std::deque<setpoint_t> setpoints_;
  pose_t cur_pose_;
  Eigen::Affine3d T_odomflu_odomned_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_odomned_odomflu_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localflu_localfrd_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localfrd_localflu_ = Eigen::Affine3d::Identity();
};
}; // namespace laser_copilot_applications

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot_applications::traj_replayer)