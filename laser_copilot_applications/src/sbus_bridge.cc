#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include "serial.h"
#include "common.hh"

namespace laser_copilot_applications {

enum class serial_type_t {
  DECODED,
  SBUS,
};

constexpr int SBUS_PACKET_SIZE = 25;
constexpr int DECODED_PACKET_SIZE = 35;
constexpr int BUFFER_SIZE = 100;
constexpr uint32_t SBUS_BAUD_RATE = 100000;
constexpr uint8_t PACKET_HEAD = 0x0f;
constexpr int CTRL_CHANNEL_NUM = 16;
constexpr int ENABLE_LIMIT = 1500;
constexpr int CHANNEL_PITCH = 1;
constexpr int CHANNEL_ROLL = 0;
constexpr int CHANNEL_THROTTLE = 2;
constexpr int CHANNEL_YAW = 3;
constexpr int MAX_READ_RETRY = 10;

struct ctrl_msg_t {
  using ptr = std::shared_ptr<ctrl_msg_t>;
  using const_ptr = std::shared_ptr<const ctrl_msg_t>;
  std::array<uint16_t, CTRL_CHANNEL_NUM> channels;

  void from_decoded_msg(const std::array<uint8_t, BUFFER_SIZE> &buf) {
    for (int i = 0; i < CTRL_CHANNEL_NUM; i++) {
      channels[i] = ((uint16_t(buf[2 * i + 1]) << 8 | uint16_t(buf[2 * i + 2])));
    }
  }

  std::string to_str() const {
    std::stringstream ss;
    ss << "channels: ";
    for (int i = 0; i < CTRL_CHANNEL_NUM; i++) {
      ss << channels[i] << "\t";
    }
    return ss.str();
  }
};

class sbus_bridge : public rclcpp::Node {
public:
  explicit sbus_bridge(const rclcpp::NodeOptions &options)
      : Node("sbus_bridge", options) {
        load_param();
        init();
  }

  ~sbus_bridge() {
    serial_close(dev_);
    serial_free(dev_);
  }

private:
  void load_param() {
    device_path_ = 
      declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
    max_speed_ = declare_parameter<double>("max_speed", 2.0);
    max_angular_speed_ =
        declare_parameter<double>("max_angular_speed", 30.0) * DEG_TO_RAD;
    read_time_ms_ = declare_parameter<int>("read_time_ms", 50);
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    serial_type_ =
        static_cast<serial_type_t>(declare_parameter<int>("serial_type", 0));
    enable_channel_ = declare_parameter<int>("enable_channel", 5) - 1;
    deadzone_ = declare_parameter<int>("deadzone", 20);
    channel_max_ = declare_parameter<int>("channel_max", 2000);
    channel_min_ = declare_parameter<int>("channel_min", 0);
    channel_mid_ = declare_parameter<int>("channel_mid", 1024);
    channel_range_ =
        std::max(channel_max_ - channel_mid_, channel_mid_ - channel_min_);
  }

  void init() {
    using namespace std::chrono_literals;
    dev_ = serial_new();
    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    if (int err = serial_open_advanced(dev_, device_path_.c_str(), baudrate_, 8,
                                       serial_parity_t::PARITY_NONE, 1, false,
                                       false)) {
      RCLCPP_FATAL_STREAM(
          get_logger(),
          "serial_open_adavanced() failed, error: " << serial_errmsg(dev_));
      exit(err);
    }
    timer_ = create_wall_timer(50ms, [this]() {
      if (!this->read_one_msg(DECODED_PACKET_SIZE)) {
        RCLCPP_WARN_STREAM(get_logger(), "read msg failed");
        return;
      }
      if (!this->is_decoded_msg_ok()) {
        RCLCPP_WARN_STREAM(get_logger(), "decoded msg check failed");
        return;
      }
      this->ctrl_msg_.from_decoded_msg(this->buf_);
      this->pub_ctrl_msg();
    });
  }

  bool read_one_msg(int pkg_size) {
    int len = 0;
    for (int retry_time = 0; retry_time < MAX_READ_RETRY; retry_time++) {
      len += serial_read(dev_, &buf_[len], pkg_size - len, read_time_ms_);
      if (buf_[0] != PACKET_HEAD) {
        len = 0;
        continue;
      }
    }
    return true;
  }

  bool is_decoded_msg_ok() {
    if (buf_[0] != 0x0f)
      return false;
    uint8_t check = buf_[1];
    for (int i = 2; i < DECODED_PACKET_SIZE - 2; i++) {
      check ^= buf_[i];
    }
    return check == buf_[DECODED_PACKET_SIZE - 1];
  }

  void pub_ctrl_msg() {
    if (ctrl_msg_.channels[enable_channel_] < ENABLE_LIMIT) {
      return;
    }
    geometry_msgs::msg::Twist msg;
    msg.linear.x = channel_to_spped(CHANNEL_PITCH);
    msg.linear.y = -channel_to_spped(CHANNEL_ROLL);
    msg.linear.z = channel_to_spped(CHANNEL_THROTTLE);
    msg.angular.z = -channel_to_angular_speed(CHANNEL_YAW);
    pub_twist_->publish(msg);
  }

  double map_channel(uint8_t id) {
    double err = ctrl_msg_.channels[id] - channel_mid_;
    return std::abs(err) < deadzone_ ? 0 : err / channel_range_;
  }

  double channel_to_spped(uint8_t id) {
    return map_channel(id) * max_speed_;
  }

  double channel_to_angular_speed(uint8_t id) {
    return map_channel(id) * max_angular_speed_;
  }

private:
  serial_t* dev_;
  std::string device_path_;
  double max_speed_;
  double max_angular_speed_;
  int read_time_ms_;
  uint32_t baudrate_;
  serial_type_t serial_type_;
  int enable_channel_;
  uint16_t deadzone_;
  uint16_t channel_max_;
  uint16_t channel_min_;
  uint16_t channel_mid_;
  double channel_range_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<uint8_t, BUFFER_SIZE> buf_;
  ctrl_msg_t ctrl_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
};
}; // namespace laser_copilot_applications

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot_applications::sbus_bridge)