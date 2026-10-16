#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "chr_controller/px4_frame_adapter.hpp"
#include "chr_controller/whole_body_kinematics.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/custom_command_position_mode.hpp"
#include "px4_msgs/msg/custom_command_velocity_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace chr_controller {
namespace {
using SteadyClock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

constexpr char kWorldFrame[] = "world_enu";
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kDobFlag = 0;
constexpr std::size_t kCustomModeFlag = 1;
constexpr std::size_t kTrajectoryFlag = 2;
constexpr std::size_t kExperimentEnableFlag = 5;
constexpr std::size_t kManualDlsFlag = 6;

bool finite_vector(const Eigen::Vector3d &value) { return value.allFinite(); }

Eigen::Quaterniond quaternion_from_message(const geometry_msgs::msg::Quaternion &q) {
  Eigen::Quaterniond result(q.w, q.x, q.y, q.z);
  if (!result.coeffs().allFinite() || result.norm() < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  return result.normalized();
}

geometry_msgs::msg::Quaternion quaternion_message(const Eigen::Quaterniond &input) {
  const auto q = input.normalized();
  geometry_msgs::msg::Quaternion result;
  result.w = q.w();
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  return result;
}

double move_toward(double current, double target, double maximum_delta) {
  return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

Eigen::Vector3d vector3_parameter(
    rclcpp::Node &node, const std::string &name, const Eigen::Vector3d &defaults) {
  const auto values = node.declare_parameter<std::vector<double>>(
    name, {defaults.x(), defaults.y(), defaults.z()});
  if (values.size() != 3 ||
      !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error(name + " must contain three finite values");
  }
  return {values[0], values[1], values[2]};
}
}  // namespace

class ChrController final : public rclcpp::Node {
 public:
  ChrController() : Node("chr_controller") {
    control_rate_hz_ = declare_parameter("control_rate_hz", 100.0);
    feedback_timeout_s_ = declare_parameter("feedback_timeout_s", 0.25);
    status_timeout_s_ = declare_parameter("status_timeout_s", 1.50);
    target_timeout_s_ = declare_parameter("target_timeout_s", 0.50);
    hover_linear_threshold_ =
      declare_parameter("hover_linear_speed_threshold_m_s", 0.15);
    hover_angular_threshold_ =
      declare_parameter("hover_angular_speed_threshold_rad_s", 0.20);
    hover_stable_duration_s_ = declare_parameter("hover_stable_duration_s", 3.0);
    maximum_base_command_speed_ =
      declare_parameter("maximum_base_command_speed_m_s", 0.15);
    maximum_yaw_command_speed_ =
      declare_parameter("maximum_yaw_command_speed_rad_s", 0.30);
    publish_velocity_feedforward_ =
      declare_parameter("publish_velocity_feedforward", false);
    maximum_velocity_feedforward_ =
      declare_parameter("maximum_velocity_feedforward_m_s", 0.30);
    maximum_yawrate_feedforward_ =
      declare_parameter("maximum_yawrate_feedforward_rad_s", 0.50);
    maximum_accepted_position_residual_ =
      declare_parameter("maximum_accepted_position_residual_m", 0.03);
    maximum_accepted_orientation_residual_ =
      declare_parameter("maximum_accepted_orientation_residual_rad", 0.15);

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"J1", "J2", "J3"});
    const auto lower = vector3_parameter(*this, "joint_lower_rad", {-1.5708, 0.0, -1.5708});
    const auto upper = vector3_parameter(
      *this, "joint_upper_rad", {1.5708, 0.5235987756, 1.5708});
    maximum_joint_speed_ = vector3_parameter(
      *this, "maximum_joint_speed_rad_s", {0.35, 0.20, 0.35});
    if (joint_names_.size() != 3) throw std::runtime_error("joint_names must have three entries");
    kinematics_ = std::make_unique<WholeBodyKinematics>(lower, upper);

    const auto trajectory_init_pose_deg = vector3_parameter(
      *this, "trajectory_init_pose_deg", {0.0, 3.0, 45.0});
    trajectory_init_pose_ = trajectory_init_pose_deg * (kPi / 180.0);
    trajectory_init_tolerance_rad_ =
      declare_parameter("trajectory_init_tolerance_deg", 2.0) * kPi / 180.0;
    trajectory_init_settle_s_ = declare_parameter("trajectory_init_settle_s", 0.5);
    trajectory_x_displacement_m_ =
      declare_parameter("trajectory_x_displacement_m", 0.50);
    trajectory_z_displacement_m_ =
      declare_parameter("trajectory_z_displacement_m", -0.50);
    trajectory_frequency_hz_ = declare_parameter("trajectory_frequency_hz", 0.03);

    ik_options_.damping = declare_parameter("dls_damping", 0.03);
    ik_options_.position_tolerance_m =
      declare_parameter("dls_position_tolerance_m", 0.0005);
    ik_options_.orientation_tolerance_rad =
      declare_parameter("dls_orientation_tolerance_rad", 0.01);
    ik_options_.orientation_weight_m_per_rad =
      declare_parameter("dls_orientation_weight_m_per_rad", 0.25);
    ik_options_.base_translation_scale =
      declare_parameter("dls_base_translation_scale", 0.35);
    ik_options_.base_yaw_scale = declare_parameter("dls_base_yaw_scale", 0.50);
    ik_options_.maximum_step = declare_parameter("dls_maximum_step", 0.12);
    const auto iterations = declare_parameter<int>("dls_maximum_iterations", 80);
    ik_options_.maximum_iterations = static_cast<std::size_t>(
      std::max<int64_t>(iterations, 1));
    ik_options_.maximum_base_displacement_m = vector3_parameter(
      *this, "maximum_base_displacement_m", {0.30, 0.30, 0.18});
    ik_options_.maximum_base_yaw_displacement_rad =
      declare_parameter("maximum_base_yaw_displacement_rad", 0.40);
    validate_parameters();

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      std::bind(&ChrController::receive_odometry, this, std::placeholders::_1));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      std::bind(&ChrController::receive_status, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/cone/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&ChrController::receive_joint_state, this, std::placeholders::_1));
    torque_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/cone/torque_enabled", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ChrController::receive_torque_state, this, std::placeholders::_1));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/chr/target/tcp_pose", 10,
      std::bind(&ChrController::receive_target, this, std::placeholders::_1));
    flags_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/control_mode_flags", 10,
      std::bind(&ChrController::receive_flags, this, std::placeholders::_1));

    position_command_pub_ =
      create_publisher<px4_msgs::msg::CustomCommandPositionMode>(
      "/fmu/in/custom_command_position_mode", 10);
    velocity_command_pub_ =
      create_publisher<px4_msgs::msg::CustomCommandVelocityMode>(
      "/fmu/in/custom_command_velocity_mode", 10);
    arm_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/cone/joint_commands", 10);
    tcp_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/chr/state/tcp_pose", 10);
    controller_state_pub_ = create_publisher<std_msgs::msg::String>(
      "/chr/controller_state", rclcpp::QoS(1).transient_local().reliable());
    logging_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/chr/data_logging_msg", rclcpp::SensorDataQoS());
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ChrController::control_step, this));
    last_step_time_ = SteadyClock::now();
    set_state(State::WaitingForFeedback);
    RCLCPP_INFO(
      get_logger(),
      "CHR controller ready. It is the sole publisher of PX4 custom commands; "
      "G runs the configured joint-init/base-diagonal trajectory without DLS; "
      "arm torque is owned by dynamixel_arm_driver.");
  }

 private:
  enum class State : int {
    WaitingForFeedback = 0,
    WaitingForHover = 1,
    WaitingForTorque = 2,
    ArmReady = 3,
    Tracking = 4,
    ArmInit = 5,
    BaseTrajectory = 6,
    Hold = 7,
    Fault = 8,
  };

  enum class TrajectoryPhase : int {
    Idle = 0,
    ArmInit = 1,
    BaseMotion = 2,
    Hold = 3,
  };

  static const char *state_name(State state) {
    switch (state) {
      case State::WaitingForFeedback: return "WAITING_FOR_FEEDBACK";
      case State::WaitingForHover: return "WAITING_FOR_HOVER";
      case State::WaitingForTorque: return "WAITING_FOR_TORQUE";
      case State::ArmReady: return "ARM_READY";
      case State::Tracking: return "TRACKING";
      case State::ArmInit: return "ARM_INIT";
      case State::BaseTrajectory: return "BASE_TRAJECTORY";
      case State::Hold: return "HOLD";
      case State::Fault: return "FAULT";
    }
    return "UNKNOWN";
  }

  void validate_parameters() const {
    if (control_rate_hz_ <= 0.0 || feedback_timeout_s_ <= 0.0 ||
        status_timeout_s_ <= 0.0 || target_timeout_s_ <= 0.0 ||
        hover_stable_duration_s_ < 0.0 || maximum_base_command_speed_ <= 0.0 ||
        maximum_yaw_command_speed_ <= 0.0 || !(maximum_joint_speed_.array() > 0.0).all() ||
        trajectory_init_tolerance_rad_ <= 0.0 || trajectory_init_settle_s_ < 0.0 ||
        trajectory_frequency_hz_ <= 0.0 ||
        !std::isfinite(trajectory_x_displacement_m_) ||
        !std::isfinite(trajectory_z_displacement_m_)) {
      throw std::runtime_error("invalid non-positive controller parameter");
    }
    if ((trajectory_init_pose_.array() < kinematics_->joint_lower().array()).any() ||
        (trajectory_init_pose_.array() > kinematics_->joint_upper().array()).any()) {
      throw std::runtime_error("trajectory_init_pose_deg exceeds configured joint limits");
    }
    const double peak_axis_speed = kPi * trajectory_frequency_hz_ * std::max(
      std::abs(trajectory_x_displacement_m_), std::abs(trajectory_z_displacement_m_));
    if (peak_axis_speed > maximum_base_command_speed_) {
      throw std::runtime_error(
        "configured diagonal trajectory exceeds maximum_base_command_speed_m_s");
    }
  }

  void receive_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr message) {
    if (message->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "vehicle_odometry pose_frame is not NED; ignoring");
      return;
    }
    Eigen::Vector3d ned_position(
      message->position[0], message->position[1], message->position[2]);
    Eigen::Vector3d ned_velocity(
      message->velocity[0], message->velocity[1], message->velocity[2]);
    Eigen::Vector3d angular_velocity(
      message->angular_velocity[0], message->angular_velocity[1],
      message->angular_velocity[2]);
    Eigen::Quaterniond ned_from_frd(
      message->q[0], message->q[1], message->q[2], message->q[3]);
    if (!finite_vector(ned_position) || !finite_vector(ned_velocity) ||
        !finite_vector(angular_velocity) || !ned_from_frd.coeffs().allFinite() ||
        ned_from_frd.norm() < 1e-6) {
      return;
    }
    base_position_enu_ = Px4FrameAdapter::ned_to_enu(ned_position);
    base_velocity_enu_ = Px4FrameAdapter::ned_to_enu(ned_velocity);
    base_orientation_enu_ = Px4FrameAdapter::ned_frd_to_enu_flu(ned_from_frd);
    angular_velocity_norm_ = angular_velocity.norm();
    last_odometry_time_ = SteadyClock::now();
    odometry_received_ = true;
  }

  void receive_status(const px4_msgs::msg::VehicleStatus::SharedPtr message) {
    armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    failsafe_ = message->failsafe || message->failure_detector_status != 0;
    last_status_time_ = SteadyClock::now();
    status_received_ = true;
  }

  void receive_joint_state(const sensor_msgs::msg::JointState::SharedPtr message) {
    JointVector updated = joint_position_;
    std::array<bool, 3> found{false, false, false};
    for (std::size_t source = 0; source < message->name.size(); ++source) {
      if (source >= message->position.size() || !std::isfinite(message->position[source])) continue;
      for (std::size_t destination = 0; destination < joint_names_.size(); ++destination) {
        if (message->name[source] == joint_names_[destination]) {
          updated[static_cast<Eigen::Index>(destination)] = message->position[source];
          found[destination] = true;
        }
      }
    }
    if (std::all_of(found.begin(), found.end(), [](bool value) { return value; })) {
      joint_position_ = updated;
      last_joint_time_ = SteadyClock::now();
      joint_state_received_ = true;
      if (!arm_command_initialized_) {
        arm_command_ = kinematics_->clamp_joints(joint_position_);
        arm_command_initialized_ = true;
      }
    }
  }

  void receive_torque_state(const std_msgs::msg::Bool::SharedPtr message) {
    const bool rising_edge = message->data && !torque_enabled_;
    torque_enabled_ = message->data;
    if (rising_edge) reset_after_torque_enable();
    if (!torque_enabled_) {
      tracking_active_ = false;
      trajectory_phase_ = TrajectoryPhase::Idle;
    }
  }

  void reset_after_torque_enable() {
    tracking_active_ = false;
    trajectory_phase_ = TrajectoryPhase::Idle;
    trajectory_init_candidate_active_ = false;
    hover_candidate_active_ = false;
    hover_stable_ = false;
    arm_command_initialized_ = false;
    RCLCPP_WARN(
      get_logger(), "torque-on detected; waiting for fresh zeroed joints and a new stable-hover window");
  }

  void receive_target(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    Eigen::Vector3d position(
      message->pose.position.x, message->pose.position.y, message->pose.position.z);
    const Eigen::Quaterniond orientation = quaternion_from_message(message->pose.orientation);
    if (!finite_vector(position) || !orientation.coeffs().allFinite()) return;
    target_pose_ = Eigen::Isometry3d::Identity();
    target_pose_.translation() = position;
    target_pose_.linear() = orientation.toRotationMatrix();
    target_received_ = true;
    last_target_time_ = SteadyClock::now();
  }

  void receive_flags(const std_msgs::msg::Int32MultiArray::SharedPtr message) {
    flags_ = message->data;
    if (flags_.size() <= kManualDlsFlag) {
      flags_.resize(kManualDlsFlag + 1, 0);
    }
    if (!custom_mode_requested()) {
      origin_valid_ = false;
      tracking_active_ = false;
      trajectory_phase_ = TrajectoryPhase::Idle;
    }
    if (!experiment_enabled()) {
      tracking_active_ = false;
      trajectory_phase_ = TrajectoryPhase::Idle;
    }
  }

  bool flag(std::size_t index) const {
    return index < flags_.size() && flags_[index] != 0;
  }
  bool custom_mode_requested() const { return flag(kCustomModeFlag); }
  bool trajectory_requested() const { return flag(kTrajectoryFlag); }
  bool experiment_enabled() const { return flag(kExperimentEnableFlag); }
  bool manual_dls_requested() const { return flag(kManualDlsFlag); }

  bool fresh(const SteadyClock::time_point &time, double timeout) const {
    return Seconds(SteadyClock::now() - time).count() <= timeout;
  }

  bool feedback_fresh() const {
    return odometry_received_ && status_received_ && joint_state_received_ &&
      fresh(last_odometry_time_, feedback_timeout_s_) &&
      fresh(last_status_time_, status_timeout_s_) &&
      fresh(last_joint_time_, feedback_timeout_s_);
  }

  bool target_fresh() const {
    return target_received_ && fresh(last_target_time_, target_timeout_s_);
  }

  bool flight_healthy() const { return feedback_fresh() && armed_ && !failsafe_; }

  void update_hover_gate() {
    const bool hovering_now = flight_healthy() && !tracking_active_ &&
      base_velocity_enu_.norm() <= hover_linear_threshold_ &&
      angular_velocity_norm_ <= hover_angular_threshold_;
    if (!hovering_now) {
      hover_candidate_active_ = false;
      hover_stable_ = false;
      return;
    }
    if (!hover_candidate_active_) {
      hover_candidate_active_ = true;
      hover_candidate_since_ = SteadyClock::now();
    }
    hover_stable_ = Seconds(SteadyClock::now() - hover_candidate_since_).count() >=
      hover_stable_duration_s_;
  }

  void capture_command_origin() {
    origin_position_enu_ = base_position_enu_;
    origin_yaw_enu_ = WholeBodyKinematics::quaternion_yaw(
      WholeBodyKinematics::level_yaw_orientation(base_orientation_enu_));
    origin_valid_ = true;
    base_offset_command_ned_.setZero();
    yaw_offset_command_ned_ = 0.0;
    previous_base_offset_command_ned_.setZero();
    previous_yaw_offset_command_ned_ = 0.0;
    RCLCPP_INFO(
      get_logger(), "captured custom-command origin ENU=[%.3f %.3f %.3f], yaw=%.3f",
      origin_position_enu_.x(), origin_position_enu_.y(), origin_position_enu_.z(),
      origin_yaw_enu_);
  }

  bool trajectory_motion_active() const {
    return trajectory_phase_ == TrajectoryPhase::ArmInit ||
      trajectory_phase_ == TrajectoryPhase::BaseMotion;
  }

  void start_predefined_trajectory() {
    tracking_active_ = false;
    trajectory_phase_ = TrajectoryPhase::ArmInit;
    trajectory_init_candidate_active_ = false;
    trajectory_base_origin_offset_ned_ = base_offset_command_ned_;
    trajectory_hold_yaw_offset_ned_ = yaw_offset_command_ned_;
    RCLCPP_WARN(
      get_logger(),
      "PREDEFINED TRAJECTORY: ARM_INIT target=[%.1f %.1f %.1f] deg",
      trajectory_init_pose_[0] * 180.0 / kPi,
      trajectory_init_pose_[1] * 180.0 / kPi,
      trajectory_init_pose_[2] * 180.0 / kPi);
  }

  void hold_predefined_trajectory() {
    if (trajectory_phase_ == TrajectoryPhase::Idle ||
        trajectory_phase_ == TrajectoryPhase::Hold) {
      return;
    }
    trajectory_phase_ = TrajectoryPhase::Hold;
    trajectory_init_candidate_active_ = false;
    RCLCPP_WARN(
      get_logger(),
      "PREDEFINED TRAJECTORY OFF: holding current base offset and arm command");
  }

  void update_predefined_trajectory(double dt) {
    if (trajectory_phase_ == TrajectoryPhase::ArmInit) {
      for (Eigen::Index joint = 0; joint < 3; ++joint) {
        arm_command_[joint] = move_toward(
          arm_command_[joint], trajectory_init_pose_[joint],
          maximum_joint_speed_[joint] * dt);
      }
      arm_command_ = kinematics_->clamp_joints(arm_command_);
      base_offset_command_ned_ = trajectory_base_origin_offset_ned_;
      yaw_offset_command_ned_ = trajectory_hold_yaw_offset_ned_;

      const bool pose_reached =
        (joint_position_ - trajectory_init_pose_).cwiseAbs().maxCoeff() <=
        trajectory_init_tolerance_rad_;
      if (!pose_reached) {
        trajectory_init_candidate_active_ = false;
        return;
      }
      if (!trajectory_init_candidate_active_) {
        trajectory_init_candidate_active_ = true;
        trajectory_init_candidate_since_ = SteadyClock::now();
        return;
      }
      if (Seconds(SteadyClock::now() - trajectory_init_candidate_since_).count() <
          trajectory_init_settle_s_) {
        return;
      }

      trajectory_phase_ = TrajectoryPhase::BaseMotion;
      trajectory_start_time_ = SteadyClock::now();
      trajectory_base_origin_offset_ned_ = base_offset_command_ned_;
      RCLCPP_WARN(
        get_logger(),
        "PREDEFINED TRAJECTORY: BASE_DIAGONAL x=%.3f m, z=%.3f m, f=%.3f Hz",
        trajectory_x_displacement_m_, trajectory_z_displacement_m_,
        trajectory_frequency_hz_);
      return;
    }

    if (trajectory_phase_ == TrajectoryPhase::BaseMotion) {
      const double elapsed = Seconds(SteadyClock::now() - trajectory_start_time_).count();
      const double phase = 2.0 * kPi * trajectory_frequency_hz_ * elapsed;
      const double blend = 0.5 * (1.0 - std::cos(phase));
      const Eigen::Vector3d displacement_enu(
        trajectory_x_displacement_m_ * blend,
        0.0,
        trajectory_z_displacement_m_ * blend);
      base_offset_command_ned_ = trajectory_base_origin_offset_ned_ +
        Px4FrameAdapter::enu_to_ned(displacement_enu);
      yaw_offset_command_ned_ = trajectory_hold_yaw_offset_ned_;
      arm_command_ = trajectory_init_pose_;
    }
  }

  bool solve_target(double dt) {
    const Eigen::Quaterniond origin_orientation =
      WholeBodyKinematics::yaw_orientation(origin_yaw_enu_);
    const auto result = kinematics_->solve_pose_dls(
      origin_position_enu_, origin_orientation, target_pose_, arm_command_, ik_options_);
    last_ik_result_ = result;
    const bool acceptable = result.base_position.allFinite() &&
      result.joint_position.allFinite() &&
      result.position_residual_m <= maximum_accepted_position_residual_ &&
      result.orientation_residual_rad <= maximum_accepted_orientation_residual_;
    if (!acceptable) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DLS rejected: position residual=%.4f m, tool-axis tilt residual=%.4f rad",
        result.position_residual_m, result.orientation_residual_rad);
      return false;
    }

    const Eigen::Vector3d desired_offset_ned = Px4FrameAdapter::enu_to_ned(
      result.base_position - origin_position_enu_);
    const double desired_yaw_ned = Px4FrameAdapter::yaw_enu_to_ned(
      WholeBodyKinematics::quaternion_yaw(result.base_orientation));
    const double origin_yaw_ned = Px4FrameAdapter::yaw_enu_to_ned(origin_yaw_enu_);
    const double desired_yaw_offset_ned = Px4FrameAdapter::wrap_angle(
      desired_yaw_ned - origin_yaw_ned);
    const double max_position_delta = maximum_base_command_speed_ * dt;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      base_offset_command_ned_[axis] = move_toward(
        base_offset_command_ned_[axis], desired_offset_ned[axis], max_position_delta);
    }
    yaw_offset_command_ned_ = move_toward(
      yaw_offset_command_ned_, desired_yaw_offset_ned,
      maximum_yaw_command_speed_ * dt);
    for (Eigen::Index joint = 0; joint < 3; ++joint) {
      arm_command_[joint] = move_toward(
        arm_command_[joint], result.joint_position[joint],
        maximum_joint_speed_[joint] * dt);
    }
    arm_command_ = kinematics_->clamp_joints(arm_command_);
    return true;
  }

  void ramp_flight_command_to_zero(double dt) {
    const double position_delta = maximum_base_command_speed_ * dt;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      base_offset_command_ned_[axis] = move_toward(
        base_offset_command_ned_[axis], 0.0, position_delta);
    }
    yaw_offset_command_ned_ = move_toward(
      yaw_offset_command_ned_, 0.0, maximum_yaw_command_speed_ * dt);
  }

  uint64_t now_microseconds() {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void publish_flight_command(double dt) {
    Eigen::Vector3d feedforward = Eigen::Vector3d::Zero();
    double yawrate = 0.0;
    if (publish_velocity_feedforward_ && dt > 1e-6) {
      feedforward = (base_offset_command_ned_ - previous_base_offset_command_ned_) / dt;
      for (Eigen::Index axis = 0; axis < 3; ++axis) {
        feedforward[axis] = std::clamp(
          feedforward[axis], -maximum_velocity_feedforward_, maximum_velocity_feedforward_);
      }
      yawrate = std::clamp(
        Px4FrameAdapter::wrap_angle(
          yaw_offset_command_ned_ - previous_yaw_offset_command_ned_) / dt,
        -maximum_yawrate_feedforward_, maximum_yawrate_feedforward_);
    }

    px4_msgs::msg::CustomCommandPositionMode position;
    position.timestamp = now_microseconds();
    position.setpoint = {
      static_cast<float>(base_offset_command_ned_.x()),
      static_cast<float>(base_offset_command_ned_.y()),
      static_cast<float>(base_offset_command_ned_.z()),
      static_cast<float>(yaw_offset_command_ned_)};
    position_command_pub_->publish(position);

    px4_msgs::msg::CustomCommandVelocityMode velocity;
    velocity.timestamp = position.timestamp;
    velocity.setpoint = {
      static_cast<float>(feedforward.x()), static_cast<float>(feedforward.y()),
      static_cast<float>(feedforward.z()), static_cast<float>(yawrate)};
    velocity_command_pub_->publish(velocity);
    previous_base_offset_command_ned_ = base_offset_command_ned_;
    previous_yaw_offset_command_ned_ = yaw_offset_command_ned_;
  }

  void publish_arm_command() {
    if (!torque_enabled_ || !arm_command_initialized_) return;
    sensor_msgs::msg::JointState command;
    command.header.stamp = now();
    command.name = joint_names_;
    command.position = {arm_command_[0], arm_command_[1], arm_command_[2]};
    arm_command_pub_->publish(command);
  }

  Eigen::Isometry3d measured_tcp_pose() const {
    return WholeBodyKinematics::tcp_in_world(
      WholeBodyKinematics::base_pose(base_position_enu_, base_orientation_enu_),
      joint_position_);
  }

  void publish_tcp_pose() {
    if (!odometry_received_ || !joint_state_received_) return;
    const auto tcp = measured_tcp_pose();
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = kWorldFrame;
    message.pose.position.x = tcp.translation().x();
    message.pose.position.y = tcp.translation().y();
    message.pose.position.z = tcp.translation().z();
    message.pose.orientation = quaternion_message(Eigen::Quaterniond(tcp.rotation()));
    tcp_pose_pub_->publish(message);
  }

  void set_state(State state) {
    if (state == state_ && state_published_) return;
    state_ = state;
    state_published_ = true;
    std_msgs::msg::String message;
    message.data = state_name(state);
    controller_state_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "controller state -> %s", message.data.c_str());
  }

  void update_state() {
    if (!feedback_fresh()) {
      set_state(State::WaitingForFeedback);
    } else if (failsafe_) {
      set_state(State::Fault);
    } else if (experiment_enabled() && !torque_enabled_) {
      set_state(State::WaitingForTorque);
    } else if (trajectory_phase_ == TrajectoryPhase::ArmInit) {
      set_state(State::ArmInit);
    } else if (trajectory_phase_ == TrajectoryPhase::BaseMotion) {
      set_state(State::BaseTrajectory);
    } else if (trajectory_phase_ == TrajectoryPhase::Hold) {
      set_state(State::Hold);
    } else if (tracking_active_) {
      set_state(State::Tracking);
    } else if (!armed_ || !hover_stable_) {
      set_state(State::WaitingForHover);
    } else if (torque_enabled_) {
      set_state(State::ArmReady);
    } else {
      set_state(State::Hold);
    }
  }

  void publish_log() {
    std_msgs::msg::Float32MultiArray log;
    log.data.reserve(53);
    const auto push = [&log](double value) { log.data.push_back(static_cast<float>(value)); };
    push(static_cast<int>(state_));
    push(armed_); push(hover_stable_); push(experiment_enabled()); push(torque_enabled_);
    push(custom_mode_requested()); push(flag(kTrajectoryFlag)); push(flag(kDobFlag));
    for (Eigen::Index i = 0; i < 3; ++i) push(base_position_enu_[i]);
    push(WholeBodyKinematics::quaternion_yaw(base_orientation_enu_));
    const Eigen::Vector3d commanded_position_enu =
      origin_position_enu_ + Px4FrameAdapter::ned_to_enu(base_offset_command_ned_);
    for (Eigen::Index i = 0; i < 3; ++i) push(commanded_position_enu[i]);
    push(origin_yaw_enu_ - yaw_offset_command_ned_);
    for (Eigen::Index i = 0; i < 3; ++i) push(joint_position_[i]);
    for (Eigen::Index i = 0; i < 3; ++i) push(arm_command_[i]);
    const auto tcp = measured_tcp_pose();
    for (Eigen::Index i = 0; i < 3; ++i) push(tcp.translation()[i]);
    const Eigen::Quaterniond tcp_q(tcp.rotation());
    push(tcp_q.x()); push(tcp_q.y()); push(tcp_q.z()); push(tcp_q.w());
    for (Eigen::Index i = 0; i < 3; ++i) push(target_pose_.translation()[i]);
    const Eigen::Quaterniond target_q(target_pose_.rotation());
    push(target_q.x()); push(target_q.y()); push(target_q.z()); push(target_q.w());
    push(last_ik_result_.position_residual_m);
    push(last_ik_result_.orientation_residual_rad);
    push(last_ik_result_.minimum_singular_value);
    push(last_ik_result_.condition_number);
    push(last_ik_result_.iterations);
    push(last_ik_result_.converged);
    for (Eigen::Index i = 0; i < 3; ++i) push(base_offset_command_ned_[i]);
    push(yaw_offset_command_ned_);
    push(feedback_fresh()); push(target_fresh());
    push(tracking_active_ || trajectory_motion_active());
    for (Eigen::Index i = 0; i < 3; ++i) push(base_velocity_enu_[i]);
    push(angular_velocity_norm_);
    logging_pub_->publish(log);
  }

  void control_step() {
    const auto step_time = SteadyClock::now();
    const double nominal_dt = 1.0 / control_rate_hz_;
    const double dt = std::clamp(Seconds(step_time - last_step_time_).count(),
      0.25 * nominal_dt, 4.0 * nominal_dt);
    last_step_time_ = step_time;
    update_hover_gate();
    if (custom_mode_requested() && odometry_received_ && !origin_valid_) {
      capture_command_origin();
    }

    const bool ready_to_start_motion = custom_mode_requested() && experiment_enabled() &&
      torque_enabled_ && hover_stable_ && flight_healthy() && origin_valid_;
    const bool safe_to_continue_motion = custom_mode_requested() && experiment_enabled() &&
      torque_enabled_ && flight_healthy() && origin_valid_;

    if (trajectory_requested()) {
      if ((trajectory_phase_ == TrajectoryPhase::Idle ||
           trajectory_phase_ == TrajectoryPhase::Hold) && ready_to_start_motion) {
        start_predefined_trajectory();
      }
    } else {
      hold_predefined_trajectory();
    }

    if (trajectory_motion_active() && !safe_to_continue_motion) {
      trajectory_phase_ = TrajectoryPhase::Idle;
      trajectory_init_candidate_active_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "predefined trajectory aborted by flight/torque/experiment safety gate");
    }

    if (trajectory_phase_ == TrajectoryPhase::Hold && manual_dls_requested()) {
      trajectory_phase_ = TrajectoryPhase::Idle;
    }

    const bool may_enter_tracking = manual_dls_requested() &&
      trajectory_phase_ == TrajectoryPhase::Idle && ready_to_start_motion && target_fresh();
    if (!tracking_active_ && may_enter_tracking) {
      tracking_active_ = true;
      RCLCPP_WARN(get_logger(), "MANUAL WHOLE-BODY DLS TRACKING ENABLED");
    }
    if (tracking_active_ &&
        (!manual_dls_requested() || trajectory_phase_ != TrajectoryPhase::Idle ||
         !safe_to_continue_motion || !target_fresh())) {
      tracking_active_ = false;
      RCLCPP_WARN(get_logger(), "tracking stopped; holding arm and ramping PX4 offset to zero");
    }

    if (trajectory_motion_active()) {
      update_predefined_trajectory(dt);
    } else if (tracking_active_) {
      if (!solve_target(dt)) tracking_active_ = false;
    }
    if (!tracking_active_ && trajectory_phase_ == TrajectoryPhase::Idle) {
      ramp_flight_command_to_zero(dt);
    }

    publish_flight_command(dt);
    publish_arm_command();
    publish_tcp_pose();
    update_state();
    if (odometry_received_ && joint_state_received_) publish_log();
  }

  double control_rate_hz_{100.0};
  double feedback_timeout_s_{0.25};
  double status_timeout_s_{1.50};
  double target_timeout_s_{0.5};
  double hover_linear_threshold_{0.15};
  double hover_angular_threshold_{0.20};
  double hover_stable_duration_s_{3.0};
  double maximum_base_command_speed_{0.15};
  double maximum_yaw_command_speed_{0.30};
  double maximum_velocity_feedforward_{0.30};
  double maximum_yawrate_feedforward_{0.50};
  double maximum_accepted_position_residual_{0.03};
  double maximum_accepted_orientation_residual_{0.15};
  double trajectory_init_tolerance_rad_{2.0 * kPi / 180.0};
  double trajectory_init_settle_s_{0.5};
  double trajectory_x_displacement_m_{0.50};
  double trajectory_z_displacement_m_{-0.50};
  double trajectory_frequency_hz_{0.03};
  bool publish_velocity_feedforward_{false};
  bool armed_{false};
  bool failsafe_{false};
  bool odometry_received_{false};
  bool status_received_{false};
  bool joint_state_received_{false};
  bool target_received_{false};
  bool torque_enabled_{false};
  bool hover_candidate_active_{false};
  bool hover_stable_{false};
  bool tracking_active_{false};
  bool origin_valid_{false};
  bool arm_command_initialized_{false};
  bool trajectory_init_candidate_active_{false};
  bool state_published_{false};
  State state_{State::WaitingForFeedback};
  TrajectoryPhase trajectory_phase_{TrajectoryPhase::Idle};

  std::vector<std::string> joint_names_;
  std::vector<int32_t> flags_{0, 0, 0, 0, 0, 0, 0};
  Eigen::Vector3d maximum_joint_speed_{0.35, 0.20, 0.35};
  JointVector trajectory_init_pose_{0.0, 3.0 * kPi / 180.0, 45.0 * kPi / 180.0};
  Eigen::Vector3d base_position_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d base_velocity_enu_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond base_orientation_enu_{Eigen::Quaterniond::Identity()};
  double angular_velocity_norm_{0.0};
  JointVector joint_position_{JointVector::Zero()};
  JointVector arm_command_{JointVector::Zero()};
  Eigen::Isometry3d target_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d origin_position_enu_{Eigen::Vector3d::Zero()};
  double origin_yaw_enu_{0.0};
  Eigen::Vector3d base_offset_command_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_base_offset_command_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d trajectory_base_origin_offset_ned_{Eigen::Vector3d::Zero()};
  double yaw_offset_command_ned_{0.0};
  double previous_yaw_offset_command_ned_{0.0};
  double trajectory_hold_yaw_offset_ned_{0.0};
  IkOptions ik_options_;
  PoseIkResult last_ik_result_;
  std::unique_ptr<WholeBodyKinematics> kinematics_;

  SteadyClock::time_point last_odometry_time_{};
  SteadyClock::time_point last_status_time_{};
  SteadyClock::time_point last_joint_time_{};
  SteadyClock::time_point last_target_time_{};
  SteadyClock::time_point hover_candidate_since_{};
  SteadyClock::time_point trajectory_init_candidate_since_{};
  SteadyClock::time_point trajectory_start_time_{};
  SteadyClock::time_point last_step_time_{};

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr flags_sub_;
  rclcpp::Publisher<px4_msgs::msg::CustomCommandPositionMode>::SharedPtr position_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::CustomCommandVelocityMode>::SharedPtr velocity_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr logging_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace chr_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<chr_controller::ChrController>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("chr_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
