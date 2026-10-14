#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr char kWorldFrame[] = "world_enu";
using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;
using Quaternion = geometry_msgs::msg::Quaternion;
using SteadyClock = std::chrono::steady_clock;

Quaternion normalized(Quaternion q) {
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm < 1e-12) {
    Quaternion identity;
    identity.w = 1.0;
    return identity;
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quaternion product(const Quaternion &a, const Quaternion &b) {
  Quaternion result;
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return normalized(result);
}

Quaternion axis_angle(int axis, double angle) {
  Quaternion result;
  result.w = std::cos(0.5 * angle);
  result.x = result.y = result.z = 0.0;
  if (axis == 0) result.x = std::sin(0.5 * angle);
  if (axis == 1) result.y = std::sin(0.5 * angle);
  if (axis == 2) result.z = std::sin(0.5 * angle);
  return result;
}

Vec2 quaternion_to_roll_pitch(const Quaternion &input) {
  const auto q = normalized(input);
  const double roll = std::atan2(
    2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch = std::asin(std::clamp(
    2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
  return {roll, pitch};
}

double quintic_ramp(double elapsed, double duration) {
  if (duration <= 1e-9) return 1.0;
  const double x = std::clamp(elapsed / duration, 0.0, 1.0);
  return 10.0 * x * x * x - 15.0 * x * x * x * x + 6.0 * x * x * x * x * x;
}
}  // namespace

class KeyboardTeleop final : public rclcpp::Node {
 public:
  KeyboardTeleop() : Node("keyboard_teleop_chr") {
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
    const bool dob_enabled_at_startup =
      declare_parameter("dob_enabled_at_startup", true);
    const bool experiment_enabled_at_startup =
      declare_parameter("experiment_enabled_at_startup", true);
    flags_[0] = dob_enabled_at_startup ? 1 : 0;
    flags_[5] = experiment_enabled_at_startup ? 1 : 0;
    translation_step_m_ = declare_parameter("translation_step_m", 0.01);
    rotation_step_rad_ = declare_parameter("rotation_step_deg", 3.0) * kPi / 180.0;
    workspace_min_ = vector_parameter("workspace_min", {-2.0, -2.0, 0.05});
    workspace_max_ = vector_parameter("workspace_max", {2.0, 2.0, 3.0});
    trajectory_amplitude_ = vector_parameter(
      "trajectory_amplitude_m", {0.04, 0.04, 0.02});
    trajectory_base_frequency_hz_ = declare_parameter("trajectory_base_frequency_hz", 0.05);
    trajectory_roll_amplitude_rad_ =
      declare_parameter("trajectory_roll_amplitude_deg", 8.0) * kPi / 180.0;
    trajectory_pitch_amplitude_rad_ =
      declare_parameter("trajectory_pitch_amplitude_deg", 6.0) * kPi / 180.0;
    trajectory_ramp_in_s_ = declare_parameter("trajectory_ramp_in_s", 3.0);
    if (publish_rate_hz_ <= 0.0 || translation_step_m_ <= 0.0 ||
        rotation_step_rad_ <= 0.0 || trajectory_base_frequency_hz_ <= 0.0 ||
        trajectory_roll_amplitude_rad_ < 0.0 || trajectory_pitch_amplitude_rad_ < 0.0 ||
        trajectory_ramp_in_s_ < 0.0) {
      throw std::runtime_error("teleop rates and steps must be positive");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      if (workspace_min_[i] >= workspace_max_[i]) {
        throw std::runtime_error("workspace_min must be below workspace_max");
      }
      if (trajectory_amplitude_[i] < 0.0) {
        throw std::runtime_error("trajectory_amplitude_m must be non-negative");
      }
    }

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/chr/target/tcp_pose", 10);
    flags_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "/control_mode_flags", 10);
    tcp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/chr/state/tcp_pose", rclcpp::SensorDataQoS(),
      std::bind(&KeyboardTeleop::receive_tcp, this, std::placeholders::_1));
    torque_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/cone/torque_enabled", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&KeyboardTeleop::receive_torque, this, std::placeholders::_1));
    controller_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/chr/controller_state", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&KeyboardTeleop::receive_controller_state, this, std::placeholders::_1));

    configure_terminal();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KeyboardTeleop::update, this));
    print_help();
    RCLCPP_WARN(
      get_logger(),
      "startup defaults: DOB=%s, arm/DLS experiment=%s; M inhibits or re-enables tracking",
      flags_[0] ? "ON" : "OFF", flags_[5] ? "AUTO-ENABLE" : "OFF");
    RCLCPP_INFO(get_logger(), "waiting for /chr/state/tcp_pose before EE commands");
  }

  ~KeyboardTeleop() override { restore_terminal(); }

 private:
  Vec3 vector_parameter(const std::string &name, const Vec3 &defaults) {
    const auto values = declare_parameter<std::vector<double>>(
      name, {defaults[0], defaults[1], defaults[2]});
    if (values.size() != 3 ||
        !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) {
      throw std::runtime_error(name + " must have three finite values");
    }
    return {values[0], values[1], values[2]};
  }

  void configure_terminal() {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error(
        "keyboard teleop requires a TTY; run 'ros2 run keyboard_teleop_chr keyboard_teleop'");
    }
    if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0) {
      throw std::runtime_error("failed to read terminal configuration");
    }
    termios raw = original_terminal_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("failed to enable raw terminal input");
    }
    terminal_configured_ = true;
  }

  void restore_terminal() {
    if (terminal_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
      terminal_configured_ = false;
    }
  }

  void receive_tcp(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    measured_pose_ = message->pose;
    measured_pose_.orientation = normalized(measured_pose_.orientation);
    if (target_reset_pending_ && SteadyClock::now() >= target_reset_not_before_) {
      target_pose_ = measured_pose_;
      trajectory_center_ = measured_pose_;
      target_reset_pending_ = false;
      target_initialized_ = true;
      RCLCPP_WARN(get_logger(), "EE target reset from post-torque zeroed joint feedback");
      return;
    }
    if (target_initialized_) return;
    target_pose_ = measured_pose_;
    target_initialized_ = true;
    RCLCPP_INFO(get_logger(), "EE target initialized from measured TCP pose");
    print_target();
  }

  void receive_torque(const std_msgs::msg::Bool::SharedPtr message) {
    const bool rising_edge = message->data && !torque_enabled_;
    torque_enabled_ = message->data;
    if (!rising_edge) return;
    trajectory_enabled_ = false;
    flags_[2] = 0;
    target_reset_pending_ = true;
    target_reset_not_before_ = SteadyClock::now() + std::chrono::milliseconds(100);
    RCLCPP_WARN(get_logger(), "torque-on detected; waiting for post-zero joint feedback");
  }

  void receive_controller_state(const std_msgs::msg::String::SharedPtr message) {
    controller_state_ = message->data;
    if (flags_[5] && !flags_[1] && controller_state_ == "ARM_READY") {
      flags_[1] = 1;
      RCLCPP_WARN(
        get_logger(), "ARM_READY received; Palletrone custom mode and DLS enabled automatically");
    }
  }

  void update() {
    char key = 0;
    while (read(STDIN_FILENO, &key, 1) > 0) handle_key(key);
    if (target_initialized_ && trajectory_enabled_) update_trajectory();
    if (target_initialized_) publish_target();
    publish_flags();
  }

  void handle_key(char raw_key) {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(raw_key)));
    if (key == 'b') { toggle_flag(0, "disturbance observer"); return; }
    if (key == 'm') { toggle_experiment(); return; }
    if (key == 'g') {
      if (!target_initialized_) {
        RCLCPP_WARN(get_logger(), "trajectory ignored until measured TCP pose arrives");
        return;
      }
      trajectory_enabled_ = !trajectory_enabled_;
      flags_[2] = trajectory_enabled_ ? 1 : 0;
      if (trajectory_enabled_) {
        trajectory_center_ = target_pose_;
        trajectory_start_ = SteadyClock::now();
      }
      RCLCPP_WARN(get_logger(), "EE trajectory %s", trajectory_enabled_ ? "ON" : "OFF (hold)");
      return;
    }

    if (!target_initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "EE command ignored: no measured TCP pose");
      return;
    }
    if (trajectory_enabled_) {
      trajectory_enabled_ = false;
      flags_[2] = 0;
      RCLCPP_INFO(get_logger(), "trajectory stopped for manual EE command");
    }
    bool changed = true;
    switch (key) {
      case 'w': target_pose_.position.x += translation_step_m_; break;
      case 's': target_pose_.position.x -= translation_step_m_; break;
      case 'a': target_pose_.position.y += translation_step_m_; break;
      case 'd': target_pose_.position.y -= translation_step_m_; break;
      case 'r': target_pose_.position.z += translation_step_m_; break;
      case 'f': target_pose_.position.z -= translation_step_m_; break;
      case 'i': rotate_local(0, rotation_step_rad_); break;
      case 'k': rotate_local(0, -rotation_step_rad_); break;
      case 'j': rotate_local(1, rotation_step_rad_); break;
      case 'l': rotate_local(1, -rotation_step_rad_); break;
      case ' ': target_pose_ = measured_pose_; break;
      case 'p': print_target(); changed = false; break;
      default: changed = false; break;
    }
    if (changed) {
      clamp_position();
      print_target();
    }
  }

  void toggle_flag(std::size_t index, const char *label) {
    flags_[index] = flags_[index] ? 0 : 1;
    RCLCPP_WARN(get_logger(), "%s = %s", label, flags_[index] ? "ON" : "OFF");
  }

  void toggle_experiment() {
    const bool enable = flags_[5] == 0;
    flags_[5] = enable ? 1 : 0;
    if (enable) {
      flags_[1] = torque_enabled_ && controller_state_ == "ARM_READY" ? 1 : 0;
      RCLCPP_WARN(
        get_logger(),
        "arm/DLS experiment = ON; waiting for arm torque and ARM_READY if needed");
      return;
    }
    flags_[1] = 0;
    flags_[2] = 0;
    trajectory_enabled_ = false;
    RCLCPP_WARN(
      get_logger(),
      "arm/DLS experiment = OFF; DLS stopped, arm torque remains owned by the driver");
  }

  void update_trajectory() {
    const double elapsed = std::chrono::duration<double>(
      SteadyClock::now() - trajectory_start_).count();
    const double ramp = quintic_ramp(elapsed, trajectory_ramp_in_s_);
    target_pose_ = trajectory_center_;
    const double phase = 2.0 * kPi * trajectory_base_frequency_hz_ * elapsed;
    target_pose_.position.x += ramp * trajectory_amplitude_[0] * std::sin(phase);
    target_pose_.position.y += ramp * trajectory_amplitude_[1] * std::sin(2.0 * phase);
    target_pose_.position.z += ramp * trajectory_amplitude_[2] * std::sin(3.0 * phase);
    const double roll = ramp * trajectory_roll_amplitude_rad_ * std::sin(2.0 * phase);
    const double pitch = ramp * trajectory_pitch_amplitude_rad_ * std::sin(3.0 * phase);
    target_pose_.orientation = product(
      product(trajectory_center_.orientation, axis_angle(0, roll)), axis_angle(1, pitch));
    clamp_position();
  }

  void rotate_local(int axis, double angle) {
    target_pose_.orientation = product(target_pose_.orientation, axis_angle(axis, angle));
  }

  void clamp_position() {
    target_pose_.position.x = std::clamp(
      target_pose_.position.x, workspace_min_[0], workspace_max_[0]);
    target_pose_.position.y = std::clamp(
      target_pose_.position.y, workspace_min_[1], workspace_max_[1]);
    target_pose_.position.z = std::clamp(
      target_pose_.position.z, workspace_min_[2], workspace_max_[2]);
  }

  void publish_target() {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = kWorldFrame;
    message.pose = target_pose_;
    target_pub_->publish(message);
  }

  void publish_flags() {
    std_msgs::msg::Int32MultiArray message;
    message.data = flags_;
    flags_pub_->publish(message);
  }

  void print_target() const {
    const auto roll_pitch = quaternion_to_roll_pitch(target_pose_.orientation);
    RCLCPP_INFO(
      get_logger(), "target xyz=[%.3f %.3f %.3f] m roll/pitch=[%.1f %.1f] deg",
      target_pose_.position.x, target_pose_.position.y, target_pose_.position.z,
      roll_pitch[0] * 180.0 / kPi, roll_pitch[1] * 180.0 / kPi);
  }

  void print_help() const {
    RCLCPP_INFO(
      get_logger(),
      "\nCHR temporary experiment keyboard\n"
      " EE xyz: W/S +X/-X, A/D +Y/-Y, R/F +Z/-Z\n"
      " EE tilt: I/K +roll/-roll, J/L +pitch/-pitch (tool-z yaw is free)\n"
      " Space: hold measured EE, P: print target\n"
      " G: xyz+roll/pitch Lissajous, B: DOB\n"
      " M: emergency DLS inhibit/re-enable (arm torque is unchanged)\n"
      " DLS auto-starts only after the controller reports ARM_READY.\n"
      " Exit with Ctrl-C after M is OFF.");
  }

  double publish_rate_hz_{50.0};
  double translation_step_m_{0.01};
  double rotation_step_rad_{0.0523598776};
  double trajectory_base_frequency_hz_{0.05};
  double trajectory_roll_amplitude_rad_{0.1396263402};
  double trajectory_pitch_amplitude_rad_{0.1047197551};
  double trajectory_ramp_in_s_{3.0};
  Vec3 workspace_min_{-2.0, -2.0, 0.05};
  Vec3 workspace_max_{2.0, 2.0, 3.0};
  Vec3 trajectory_amplitude_{0.04, 0.04, 0.02};
  std::vector<int32_t> flags_{0, 0, 0, 0, 0, 0};
  bool terminal_configured_{false};
  bool target_initialized_{false};
  bool trajectory_enabled_{false};
  bool torque_enabled_{false};
  bool target_reset_pending_{false};
  termios original_terminal_{};
  std::string controller_state_;
  geometry_msgs::msg::Pose measured_pose_;
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Pose trajectory_center_;
  SteadyClock::time_point trajectory_start_{};
  SteadyClock::time_point target_reset_not_before_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr flags_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_teleop_chr"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
