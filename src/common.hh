#pragma once
#include <Eigen/Eigen>

namespace laser_copilot {
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr int GROUP_NUM = 72;
constexpr int SEARCH_RANGE = 6; // search 30° on each side
constexpr float RAD_INC = 2 * M_PI / GROUP_NUM;
constexpr float MAX_DIST = std::numeric_limits<std::uint16_t>::max();

inline int id(double x, double y) {
  return static_cast<int>((atan2(y, x) * RAD_TO_DEG + (y < 0 ? 360.0 : 0.0)) /
                          5) % GROUP_NUM;
}

inline int wrap_id(int id) {
  id = id % GROUP_NUM;
  while (id < 0) {
    id += GROUP_NUM;
  }
  return id;
}

inline float dist(double x, double y) { return std::sqrt(x * x + y * y); }

inline double quaternion_to_yaw(Eigen::Quaterniond q) {
  Eigen::Vector3d vec = q * Eigen::Vector3d::UnitX();
  return std::atan2(vec[1], vec[0]);
}

struct velodyne_point_v1_t {
  alignas(2) float x;
  alignas(2) float y;
  alignas(2) float z;
  alignas(2) float intensity;
  alignas(2) uint16_t ring;
};

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
  uint64_t stamp; // nanosec
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
  setpoint_t(Eigen::Affine3d t) : setpoint_t(
    Eigen::Vector3d{t.translation()}, 
    Eigen::Quaterniond{t.linear()}){}
  friend std::ostream &operator<<(std::ostream &os, const setpoint_t &sp) {
    os << sp.position.transpose() << " " << sp.yaw / M_PI * 180.0;
    return os;
  }
  Eigen::Affine3d to_affine() const {
    Eigen::Affine3d ret = Eigen::Affine3d::Identity();
    ret.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    ret.translation() = position;
    return ret;
  }
};

setpoint_t flu_to_frd(const setpoint_t& sp) {
  static bool is_init_transformations = false;
  static Eigen::Affine3d T_flu_frd;
  if (!is_init_transformations) {
    T_flu_frd = Eigen::Affine3d::Identity();
    T_flu_frd.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    is_init_transformations = true;
  }
  return {T_flu_frd * sp.to_affine()};
}

/**
 * @brief transform setpoint from flu frame to ned frame, the secquence is 
 *        flu -> frd -> ned
 * @param sp: setpoint in flu frame
 * @param yaw_frd_ned: radius yaw angle bewteen from ned to frd
*/
setpoint_t flu_to_ned(const setpoint_t& sp, double yaw_frd_ned) {
  static bool is_init_transformations = false;
  static Eigen::Affine3d T_flu_ned;
  if (!is_init_transformations) {
    Eigen::Affine3d T_flu_frd = Eigen::Affine3d::Identity();
    T_flu_frd.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Affine3d T_frd_ned = Eigen::Affine3d::Identity();
    T_frd_ned.linear() =
        Eigen::AngleAxisd(yaw_frd_ned, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_flu_ned = T_frd_ned * T_flu_frd;
    is_init_transformations = true;
  }
  return {T_flu_ned * sp.to_affine()};
}

/**
 * @brief transform setpoint from flu frame to ned frame, the secquence is
 *        ned -> frd -> flu
 * @param sp: setpoint in ned frame
 * @param yaw_frd_ned: radius yaw angle bewteen from ned to frd
 */
setpoint_t ned_to_flu(const setpoint_t &sp, double yaw_frd_ned) {
  static bool is_init_transformations = false;
  static Eigen::Affine3d T_ned_flu;
  if (!is_init_transformations) {
    Eigen::Affine3d T_flu_frd = Eigen::Affine3d::Identity();
    T_flu_frd.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Affine3d T_frd_ned = Eigen::Affine3d::Identity();
    T_frd_ned.linear() =
        Eigen::AngleAxisd(yaw_frd_ned, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    T_ned_flu = (T_frd_ned * T_flu_frd).inverse();
    is_init_transformations = true;
  }
  return {T_ned_flu * sp.to_affine()};
}

Eigen::Affine3d create_affine_flu_to_ned(double yaw_frd_ned) {
  Eigen::Affine3d T_flu_frd = Eigen::Affine3d::Identity();
  T_flu_frd.linear() =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Affine3d T_frd_ned = Eigen::Affine3d::Identity();
  T_frd_ned.linear() = Eigen::AngleAxisd(yaw_frd_ned, Eigen::Vector3d::UnitZ())
                           .toRotationMatrix();
  return T_frd_ned * T_flu_frd;
}
};