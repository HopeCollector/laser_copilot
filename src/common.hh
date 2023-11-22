#pragma once
#include <Eigen/Eigen>

namespace laser_copilot {
double quaternion_to_yaw(Eigen::Quaterniond q) {
  Eigen::Vector3d vec = q * Eigen::Vector3d::UnitX();
  return std::atan2(vec[1], vec[0]);
}

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
  float yaw = 0.0;
};

struct setpoint_t {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  float yaw = 0.0;
  setpoint_t() {}
  setpoint_t(Eigen::Vector3d p, Eigen::Quaterniond q) {
    position = p;
    yaw = quaternion_to_yaw(q);
  }
  friend std::ostream &operator<<(std::ostream &os, const setpoint_t &sp) {
    os << sp.position.transpose() << " " << sp.yaw / M_PI * 180.0;
    return os;
  }
};
};