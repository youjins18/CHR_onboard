#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <px4_msgs/msg/custom_command_position_mode.hpp>
#include <px4_msgs/msg/custom_command_velocity_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher")
  {
    mode1_z_displacement_ =
      declare_parameter<double>("mode1_z_displacement_m", -0.40);
    mode2_x_displacement_ =
      declare_parameter<double>("mode2_x_displacement_m", 0.40);
    mode2_z_displacement_ =
      declare_parameter<double>("mode2_z_displacement_m", -0.40);
    mode3_x_displacement_ =
      declare_parameter<double>("mode3_x_displacement_m", 0.40);
    trajectory_frequency_ =
      declare_parameter<double>("trajectory_frequency_hz", 0.05);
    j1_link_length_ = declare_parameter<double>("j1_link_length_m", 0.333);

    command_position_.setZero();
    command_velocity_.setZero();

    command_position_pub_ =
      create_publisher<px4_msgs::msg::CustomCommandPositionMode>(
      "/fmu/in/custom_command_position_mode", 10);
    command_velocity_pub_ =
      create_publisher<px4_msgs::msg::CustomCommandVelocityMode>(
      "/fmu/in/custom_command_velocity_mode", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 6), qos_profile);

    flags_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "control_mode_flags", qos,
      std::bind(&TrajectoryPublisher::receive_flags, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      std::bind(&TrajectoryPublisher::receive_odometry, this, std::placeholders::_1));
    j1_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "j1_angle", qos,
      std::bind(&TrajectoryPublisher::receive_j1_angle, this, std::placeholders::_1));

    mode_start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_),
      std::bind(&TrajectoryPublisher::update, this));
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  enum class Mode {
    Idle,
    Legacy,
    J1Swing,
    Vertical,
    Diagonal,
    Horizontal,
    Hold
  };

  struct Reference {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
  };

  static double wrap_pi(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double yaw_from_quaternion(const std::array<float, 4> &q)
  {
    return std::atan2(
      2.0 * (q[0] * q[3] + q[1] * q[2]),
      1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  }

  Mode requested_mode(const std::vector<int32_t> &flags) const
  {
    if (flags.size() < 9 || flags[1] == 0) return Mode::Idle;
    if (flags[2]) return Mode::Legacy;
    if (flags[3]) return Mode::J1Swing;
    if (flags[5]) return Mode::Vertical;
    if (flags[6]) return Mode::Diagonal;
    if (flags[7]) return Mode::Horizontal;
    if (flags[8]) return Mode::Hold;
    return Mode::Idle;
  }

  void receive_flags(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    const bool custom_enabled = msg->data.size() >= 2 && msg->data[1] != 0;
    const Mode next_mode = requested_mode(msg->data);

    if (custom_enabled && !custom_enabled_) {
      capture_custom_origin();
    }

    custom_enabled_ = custom_enabled;
    if (next_mode != mode_) {
      mode_ = next_mode;
      mode_start_ = std::chrono::steady_clock::now();
      capture_mode_origin();
      j1_reference_angle_ = j1_angle_;
    }

    if (!custom_enabled_) {
      custom_origin_valid_ = false;
    }
  }

  void receive_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) return;
    if (!std::isfinite(msg->position[0]) ||
        !std::isfinite(msg->position[1]) ||
        !std::isfinite(msg->position[2])) {
      return;
    }

    odometry_position_ <<
      msg->position[0], msg->position[1], msg->position[2];
    odometry_yaw_ = yaw_from_quaternion(msg->q);
    odometry_received_ = true;

    if (custom_enabled_ && !custom_origin_valid_) {
      capture_custom_origin();
      capture_mode_origin();
    }
  }

  void receive_j1_angle(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    if (j1_angle_received_) {
      const double dt =
        std::chrono::duration<double>(now - last_j1_angle_time_).count();
      if (dt > 1.0e-6) {
        j1_angular_velocity_ = (msg->data - j1_angle_) / dt;
      }
    }
    j1_angle_ = msg->data;
    j1_angle_received_ = true;
    last_j1_angle_time_ = now;
  }

  void capture_custom_origin()
  {
    if (!odometry_received_) return;
    custom_origin_position_ = odometry_position_;
    custom_origin_yaw_ = odometry_yaw_;
    custom_origin_valid_ = true;
  }

  void capture_mode_origin()
  {
    if (custom_origin_valid_ && odometry_received_) {
      mode_origin_position_ = odometry_position_ - custom_origin_position_;
      mode_origin_yaw_ = wrap_pi(odometry_yaw_ - custom_origin_yaw_);
    } else {
      mode_origin_position_ = command_position_.head<3>();
      mode_origin_yaw_ = command_position_(3);
    }
  }

  Reference legacy_reference(double time) const
  {
    constexpr double fx = 0.30;
    constexpr double fy = 0.15;
    constexpr double fz = 0.60;
    constexpr double ax = -0.15;
    constexpr double ay = 0.35;
    constexpr double az = 0.15;

    const double wx = 2.0 * kPi * fx;
    const double wy = 2.0 * kPi * fy;
    const double wz = 2.0 * kPi * fz;
    const double ramp_time = 3.0;
    const double tau = std::clamp(time / ramp_time, 0.0, 1.0);
    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    const double tau4 = tau3 * tau;
    const double tau5 = tau4 * tau;
    const double scale = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
    const double scale_dot = time < ramp_time ?
      (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / ramp_time : 0.0;

    Reference reference;
    const double x = ax * std::sin(wx * time);
    const double y = ay * std::sin(wy * time);
    const double z = 0.5 * az * (std::cos(wz * time) - 1.0);
    const double vx = ax * wx * std::cos(wx * time);
    const double vy = ay * wy * std::cos(wy * time);
    const double vz = -0.5 * az * wz * std::sin(wz * time);

    reference.x = scale * x;
    reference.y = scale * y;
    reference.z = scale * z;
    reference.vx = scale * vx + scale_dot * x;
    reference.vy = scale * vy + scale_dot * y;
    reference.vz = scale * vz + scale_dot * z;
    return reference;
  }

  Reference periodic_reference(Mode mode, double time) const
  {
    const double omega = 2.0 * kPi * trajectory_frequency_;
    const double blend = 0.5 * (1.0 - std::cos(omega * time));
    const double blend_dot = 0.5 * omega * std::sin(omega * time);

    Reference reference;
    if (mode == Mode::Vertical) {
      reference.z = -mode1_z_displacement_ * blend;
      reference.vz = -mode1_z_displacement_ * blend_dot;
    } else if (mode == Mode::Diagonal) {
      reference.x = mode2_x_displacement_ * blend;
      reference.vx = mode2_x_displacement_ * blend_dot;
      reference.z = -mode2_z_displacement_ * blend;
      reference.vz = -mode2_z_displacement_ * blend_dot;
    } else if (mode == Mode::Horizontal) {
      reference.x = mode3_x_displacement_ * blend;
      reference.vx = mode3_x_displacement_ * blend_dot;
    }
    return reference;
  }

  Reference j1_swing_reference() const
  {
    Reference reference;
    reference.x = j1_link_length_ *
      (std::sin(j1_reference_angle_) - std::sin(j1_angle_));
    reference.z = j1_link_length_ *
      (std::cos(j1_reference_angle_) - std::cos(j1_angle_));
    reference.vx = -j1_link_length_ * std::cos(j1_angle_) *
      j1_angular_velocity_;
    reference.vz = j1_link_length_ * std::sin(j1_angle_) *
      j1_angular_velocity_;
    return reference;
  }

  void update()
  {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - mode_start_).count();

    Reference reference;
    if (mode_ == Mode::Legacy) {
      reference = legacy_reference(elapsed);
      command_position_ <<
        reference.x, reference.y, reference.z, 0.0;
    } else if (mode_ == Mode::J1Swing) {
      reference = j1_swing_reference();
      command_position_ <<
        mode_origin_position_.x() + reference.x,
        mode_origin_position_.y(),
        mode_origin_position_.z() + reference.z,
        mode_origin_yaw_;
    } else if (
      mode_ == Mode::Vertical ||
      mode_ == Mode::Diagonal ||
      mode_ == Mode::Horizontal)
    {
      reference = periodic_reference(mode_, elapsed);
      command_position_ <<
        mode_origin_position_.x() + reference.x,
        mode_origin_position_.y(),
        mode_origin_position_.z() + reference.z,
        mode_origin_yaw_;
    } else if (mode_ == Mode::Hold) {
      command_position_ <<
        mode_origin_position_.x(),
        mode_origin_position_.y(),
        mode_origin_position_.z(),
        mode_origin_yaw_;
    } else {
      command_position_.setZero();
    }

    command_velocity_ <<
      reference.vx, reference.vy, reference.vz, 0.0;

    px4_msgs::msg::CustomCommandPositionMode position;
    position.timestamp = now_microseconds();
    position.setpoint = {
      static_cast<float>(command_position_(0)),
      static_cast<float>(command_position_(1)),
      static_cast<float>(command_position_(2)),
      static_cast<float>(command_position_(3))};
    command_position_pub_->publish(position);

    px4_msgs::msg::CustomCommandVelocityMode velocity;
    velocity.timestamp = position.timestamp;
    velocity.setpoint = {
      static_cast<float>(command_velocity_(0)),
      static_cast<float>(command_velocity_(1)),
      static_cast<float>(command_velocity_(2)),
      static_cast<float>(command_velocity_(3))};
    command_velocity_pub_->publish(velocity);
  }

  uint64_t now_microseconds() const
  {
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    return static_cast<uint64_t>(steady_clock.now().nanoseconds() / 1000ULL);
  }

  const double timer_period_{0.005};
  double mode1_z_displacement_{-0.40};
  double mode2_x_displacement_{0.40};
  double mode2_z_displacement_{-0.40};
  double mode3_x_displacement_{0.40};
  double trajectory_frequency_{0.05};
  double j1_link_length_{0.333};

  Mode mode_{Mode::Idle};
  bool custom_enabled_{false};
  bool odometry_received_{false};
  bool custom_origin_valid_{false};
  bool j1_angle_received_{false};
  double odometry_yaw_{0.0};
  double custom_origin_yaw_{0.0};
  double mode_origin_yaw_{0.0};
  double j1_angle_{0.0};
  double j1_reference_angle_{0.0};
  double j1_angular_velocity_{0.0};
  Eigen::Vector3d odometry_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d custom_origin_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mode_origin_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector4d command_position_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d command_velocity_{Eigen::Vector4d::Zero()};
  std::chrono::steady_clock::time_point mode_start_;
  std::chrono::steady_clock::time_point last_j1_angle_time_;

  rclcpp::Publisher<px4_msgs::msg::CustomCommandPositionMode>::SharedPtr
    command_position_pub_;
  rclcpp::Publisher<px4_msgs::msg::CustomCommandVelocityMode>::SharedPtr
    command_velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr flags_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j1_angle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
