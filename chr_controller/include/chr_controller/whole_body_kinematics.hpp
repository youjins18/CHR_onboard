#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>

namespace chr_controller {

using JointVector = Eigen::Vector3d;

struct IkOptions {
  double damping{0.03};
  double position_tolerance_m{1e-4};
  double orientation_tolerance_rad{0.01};
  double orientation_weight_m_per_rad{0.25};
  double base_translation_scale{0.35};
  double base_yaw_scale{0.5};
  double maximum_step{0.12};
  Eigen::Vector3d maximum_base_displacement_m{0.35, 0.35, 0.20};
  double maximum_base_yaw_displacement_rad{0.45};
  std::size_t maximum_iterations{80};
};

struct PoseIkResult {
  Eigen::Vector3d base_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond base_orientation{Eigen::Quaterniond::Identity()};
  JointVector joint_position{JointVector::Zero()};
  double position_residual_m{0.0};
  double orientation_residual_rad{0.0};
  double minimum_singular_value{0.0};
  double condition_number{0.0};
  std::size_t iterations{0};
  bool converged{false};
};

class WholeBodyKinematics {
 public:
  WholeBodyKinematics(
    const JointVector &joint_lower = JointVector(-1.5708, 0.0, -1.5708),
    const JointVector &joint_upper = JointVector(1.5708, 0.5235987756, 1.5708));

  JointVector clamp_joints(const JointVector &joint_position) const;
  static double wrap_angle(double angle);
  static double quaternion_yaw(const Eigen::Quaterniond &orientation);
  static Eigen::Quaterniond yaw_orientation(double yaw);
  static Eigen::Quaterniond level_yaw_orientation(const Eigen::Quaterniond &orientation);
  static Eigen::Isometry3d base_pose(
    const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

  /// Hardware-calibrated chain copied from the real2sim model.
  static Eigen::Isometry3d tcp_in_world(
    const Eigen::Isometry3d &world_from_base, const JointVector &joint_position);

  /// Solve TCP position and tool-z tilt with q=[base x,y,z,yaw,J1,J2,J3].
  /// Rotation about the TCP local z axis is intentionally unconstrained.
  PoseIkResult solve_pose_dls(
    const Eigen::Vector3d &seed_base_position,
    const Eigen::Quaterniond &seed_base_orientation,
    const Eigen::Isometry3d &target_pose,
    const JointVector &seed_joint_position,
    const IkOptions &options) const;

  const JointVector &joint_lower() const { return joint_lower_; }
  const JointVector &joint_upper() const { return joint_upper_; }

 private:
  JointVector joint_lower_;
  JointVector joint_upper_;
};

}  // namespace chr_controller
