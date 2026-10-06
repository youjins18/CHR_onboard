#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace {
constexpr uint16_t kAddrOperatingMode = 11;
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrGoalPosition = 116;
constexpr uint16_t kAddrPresentPosition = 132;
constexpr uint16_t kPositionLength = 4;
constexpr uint8_t kExtendedPositionMode = 4;
constexpr int32_t kTicksPerRevolution = 4096;
constexpr int32_t kExtendedPositionTickLimit = 1048575;
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

class DynamixelArmDriver final : public rclcpp::Node {
 public:
  DynamixelArmDriver() : Node("dynamixel_arm_driver") {
    usb_port_ = declare_parameter<std::string>("usb_port", "/dev/dynamixel");
    baudrate_ = declare_parameter<int>("baudrate", 4500000);
    protocol_version_ = declare_parameter<double>("protocol_version", 2.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    torque_on_at_startup_ = declare_parameter<bool>("torque_on_at_startup", false);
    capture_zero_on_enable_ =
      declare_parameter<bool>("capture_zero_on_torque_enable", true);
    disable_torque_on_shutdown_ =
      declare_parameter<bool>("disable_torque_on_shutdown", true);

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"J1", "J2", "J3", "J4"});
    const auto ids = declare_parameter<std::vector<int64_t>>(
      "motor_ids", {1, 2, 3, 4});
    joint_direction_ = declare_parameter<std::vector<double>>(
      "joint_direction", {1.0, 1.0, 1.0, 1.0});
    joint_offset_rad_ = declare_parameter<std::vector<double>>(
      "joint_offset_rad", {0.0, 0.0, 0.0, 0.0});
    joint_min_rad_ = declare_parameter<std::vector<double>>(
      "joint_min_rad", {-1.5708, 0.0, -1.5708, -3.14159});
    joint_max_rad_ = declare_parameter<std::vector<double>>(
      "joint_max_rad", {1.5708, 0.5235987756, 1.5708, 3.14159});

    validate_parameters(ids, publish_rate_hz);
    for (const auto id : ids) motor_ids_.push_back(static_cast<uint8_t>(id));
    zero_ticks_.assign(motor_ids_.size(), 0);
    online_.assign(motor_ids_.size(), false);

    port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
    if (!port_handler_->openPort()) {
      throw std::runtime_error("failed to open Dynamixel port " + usb_port_);
    }
    if (!port_handler_->setBaudRate(baudrate_)) {
      port_handler_->closePort();
      throw std::runtime_error("failed to set Dynamixel baudrate");
    }

    initialize_bus();
    sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, kAddrGoalPosition, kPositionLength);
    sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, kAddrPresentPosition, kPositionLength);
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      if (online_[i]) sync_read_->addParam(motor_ids_[i]);
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/cone/joint_states", rclcpp::SensorDataQoS());
    torque_state_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/cone/torque_enabled", rclcpp::QoS(1).transient_local().reliable());
    joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/cone/joint_commands", 10,
      std::bind(&DynamixelArmDriver::receive_joint_command, this, std::placeholders::_1));
    torque_service_ = create_service<std_srvs::srv::SetBool>(
      "/cone/set_torque",
      std::bind(
        &DynamixelArmDriver::set_torque_service, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DynamixelArmDriver::publish_state, this));

    publish_torque_state();
    if (torque_on_at_startup_ && !set_torque(true)) {
      throw std::runtime_error("failed to enable startup torque");
    }
    RCLCPP_INFO(
      get_logger(), "Arm bus ready on %s @ %d bps; torque starts %s",
      usb_port_.c_str(), baudrate_, torque_enabled_ ? "ON" : "OFF");
  }

  ~DynamixelArmDriver() override {
    if (port_handler_ != nullptr) {
      if (disable_torque_on_shutdown_) set_torque(false);
      port_handler_->closePort();
    }
  }

 private:
  void validate_parameters(const std::vector<int64_t> &ids, double publish_rate_hz) {
    const std::size_t count = joint_names_.size();
    if (count == 0 || ids.size() != count || joint_direction_.size() != count ||
        joint_offset_rad_.size() != count || joint_min_rad_.size() != count ||
        joint_max_rad_.size() != count) {
      throw std::runtime_error("all per-joint parameter arrays must have the same non-zero size");
    }
    if (publish_rate_hz <= 0.0) throw std::runtime_error("publish_rate_hz must be positive");
    for (std::size_t i = 0; i < count; ++i) {
      if (ids[i] < 0 || ids[i] > 252) throw std::runtime_error("invalid Dynamixel ID");
      if (std::abs(joint_direction_[i]) < 0.5) {
        throw std::runtime_error("joint_direction must be +1 or -1");
      }
      joint_direction_[i] = joint_direction_[i] >= 0.0 ? 1.0 : -1.0;
      if (joint_min_rad_[i] > joint_max_rad_[i]) {
        throw std::runtime_error("joint_min_rad exceeds joint_max_rad");
      }
    }
  }

  bool write_byte(uint8_t id, uint16_t address, uint8_t value, const char *operation) {
    uint8_t device_error = 0;
    const int result = packet_handler_->write1ByteTxRx(
      port_handler_, id, address, value, &device_error);
    if (result != COMM_SUCCESS || device_error != 0) {
      RCLCPP_ERROR(
        get_logger(), "ID %u %s failed: %s / %s", id, operation,
        packet_handler_->getTxRxResult(result),
        packet_handler_->getRxPacketError(device_error));
      return false;
    }
    return true;
  }

  bool read_position(std::size_t index, int32_t &ticks) {
    uint8_t device_error = 0;
    uint32_t raw = 0;
    const int result = packet_handler_->read4ByteTxRx(
      port_handler_, motor_ids_[index], kAddrPresentPosition, &raw, &device_error);
    if (result != COMM_SUCCESS || device_error != 0) {
      RCLCPP_ERROR(
        get_logger(), "ID %u position read failed: %s / %s", motor_ids_[index],
        packet_handler_->getTxRxResult(result),
        packet_handler_->getRxPacketError(device_error));
      return false;
    }
    ticks = static_cast<int32_t>(raw);
    return true;
  }

  bool write_goal_ticks(std::size_t index, int32_t ticks) {
    uint8_t device_error = 0;
    const int result = packet_handler_->write4ByteTxRx(
      port_handler_, motor_ids_[index], kAddrGoalPosition,
      static_cast<uint32_t>(ticks), &device_error);
    if (result != COMM_SUCCESS || device_error != 0) {
      RCLCPP_ERROR(
        get_logger(), "ID %u goal write failed: %s / %s", motor_ids_[index],
        packet_handler_->getTxRxResult(result),
        packet_handler_->getRxPacketError(device_error));
      return false;
    }
    return true;
  }

  void initialize_bus() {
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      uint8_t device_error = 0;
      const int ping_result = packet_handler_->ping(port_handler_, motor_ids_[i], &device_error);
      if (ping_result != COMM_SUCCESS || device_error != 0) {
        RCLCPP_ERROR(get_logger(), "ID %u (%s) not available", motor_ids_[i], joint_names_[i].c_str());
        continue;
      }
      if (!write_byte(motor_ids_[i], kAddrTorqueEnable, 0, "torque disable") ||
          !write_byte(motor_ids_[i], kAddrOperatingMode, kExtendedPositionMode, "mode set")) {
        continue;
      }
      int32_t position = 0;
      if (!read_position(i, position)) continue;
      zero_ticks_[i] = position;
      online_[i] = true;
      RCLCPP_INFO(
        get_logger(), "ID %u (%s) online, zero candidate=%d",
        motor_ids_[i], joint_names_[i].c_str(), position);
    }
    if (!std::all_of(online_.begin(), online_.end(), [](bool online) { return online; })) {
      throw std::runtime_error("not all configured arm motors are online");
    }
  }

  bool set_torque(bool enable) {
    if (enable == torque_enabled_) return true;
    if (enable) {
      std::vector<int32_t> present(motor_ids_.size(), 0);
      for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
        if (!read_position(i, present[i])) return false;
      }
      for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
        if (capture_zero_on_enable_) zero_ticks_[i] = present[i];
        if (!write_goal_ticks(i, present[i])) return false;
      }
      std::size_t enabled_count = 0;
      for (; enabled_count < motor_ids_.size(); ++enabled_count) {
        if (!write_byte(
              motor_ids_[enabled_count], kAddrTorqueEnable, 1, "torque enable")) {
          for (std::size_t rollback = 0; rollback < enabled_count; ++rollback) {
            write_byte(motor_ids_[rollback], kAddrTorqueEnable, 0, "torque rollback");
          }
          return false;
        }
      }
      torque_enabled_ = true;
      RCLCPP_WARN(get_logger(), "ARM TORQUE ENABLED; current hanging pose is q=[0,0,0,0]");
    } else {
      bool success = true;
      for (const uint8_t id : motor_ids_) {
        success = write_byte(id, kAddrTorqueEnable, 0, "torque disable") && success;
      }
      if (!success) return false;
      torque_enabled_ = false;
      RCLCPP_WARN(get_logger(), "ARM TORQUE DISABLED");
    }
    publish_torque_state();
    return true;
  }

  void set_torque_service(
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
    response->success = set_torque(request->data);
    response->message = response->success ?
      (request->data ? "arm torque enabled" : "arm torque disabled") :
      "Dynamixel communication failed; inspect driver log";
  }

  int32_t radians_to_ticks(double radians, std::size_t index) const {
    const double motor_radians =
      joint_direction_[index] * (radians + joint_offset_rad_[index]);
    const auto delta = static_cast<int32_t>(
      std::llround(motor_radians * static_cast<double>(kTicksPerRevolution) / kTwoPi));
    return std::clamp(
      zero_ticks_[index] + delta,
      -kExtendedPositionTickLimit, kExtendedPositionTickLimit);
  }

  double ticks_to_radians(int32_t ticks, std::size_t index) const {
    const double motor_radians =
      static_cast<double>(ticks - zero_ticks_[index]) * kTwoPi /
      static_cast<double>(kTicksPerRevolution);
    return joint_direction_[index] * motor_radians - joint_offset_rad_[index];
  }

  void receive_joint_command(const sensor_msgs::msg::JointState::SharedPtr message) {
    if (!torque_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "joint command ignored while arm torque is OFF");
      return;
    }
    sync_write_->clearParam();
    bool any = false;
    for (std::size_t source = 0; source < message->name.size(); ++source) {
      if (source >= message->position.size() || !std::isfinite(message->position[source])) continue;
      const auto destination = std::find(
        joint_names_.begin(), joint_names_.end(), message->name[source]);
      if (destination == joint_names_.end()) continue;
      const std::size_t index = static_cast<std::size_t>(destination - joint_names_.begin());
      const double requested = message->position[source];
      const double limited = std::clamp(requested, joint_min_rad_[index], joint_max_rad_[index]);
      if (std::abs(limited - requested) > 1e-9) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s command %.4f rad clamped to %.4f rad", joint_names_[index].c_str(),
          requested, limited);
      }
      const int32_t ticks = radians_to_ticks(limited, index);
      uint8_t bytes[4] = {
        DXL_LOBYTE(DXL_LOWORD(ticks)), DXL_HIBYTE(DXL_LOWORD(ticks)),
        DXL_LOBYTE(DXL_HIWORD(ticks)), DXL_HIBYTE(DXL_HIWORD(ticks))};
      if (!sync_write_->addParam(motor_ids_[index], bytes)) {
        RCLCPP_ERROR(get_logger(), "failed to stage command for ID %u", motor_ids_[index]);
        continue;
      }
      any = true;
    }
    if (any) {
      const int result = sync_write_->txPacket();
      if (result != COMM_SUCCESS) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "sync write failed: %s",
          packet_handler_->getTxRxResult(result));
      }
    }
  }

  void publish_torque_state() {
    if (!torque_state_pub_) return;
    std_msgs::msg::Bool state;
    state.data = torque_enabled_;
    torque_state_pub_->publish(state);
  }

  void publish_state() {
    const int result = sync_read_->txRxPacket();
    if (result != COMM_SUCCESS) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "sync read failed: %s",
        packet_handler_->getTxRxResult(result));
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      if (!sync_read_->isAvailable(motor_ids_[i], kAddrPresentPosition, kPositionLength)) continue;
      const int32_t ticks = static_cast<int32_t>(
        sync_read_->getData(motor_ids_[i], kAddrPresentPosition, kPositionLength));
      message.name.push_back(joint_names_[i]);
      message.position.push_back(ticks_to_radians(ticks, i));
    }
    joint_state_pub_->publish(message);
    publish_torque_state();
  }

  std::string usb_port_;
  int baudrate_{4500000};
  double protocol_version_{2.0};
  bool torque_on_at_startup_{false};
  bool capture_zero_on_enable_{true};
  bool disable_torque_on_shutdown_{true};
  bool torque_enabled_{false};
  std::vector<std::string> joint_names_;
  std::vector<uint8_t> motor_ids_;
  std::vector<double> joint_direction_;
  std::vector<double> joint_offset_rad_;
  std::vector<double> joint_min_rad_;
  std::vector<double> joint_max_rad_;
  std::vector<int32_t> zero_ticks_;
  std::vector<bool> online_;

  dynamixel::PortHandler *port_handler_{nullptr};
  dynamixel::PacketHandler *packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_;
  std::unique_ptr<dynamixel::GroupSyncRead> sync_read_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DynamixelArmDriver>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("dynamixel_arm_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
