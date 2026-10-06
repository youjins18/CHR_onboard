#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace chr_controller {

/// Explicit conversion between PX4 NED/FRD and the kinematics ENU/FLU world.
class Px4FrameAdapter {
 public:
  static Eigen::Vector3d ned_to_enu(const Eigen::Vector3d &ned);
  static Eigen::Vector3d enu_to_ned(const Eigen::Vector3d &enu);
  static Eigen::Quaterniond ned_frd_to_enu_flu(const Eigen::Quaterniond &ned_from_frd);
  static Eigen::Quaterniond enu_flu_to_ned_frd(const Eigen::Quaterniond &enu_from_flu);
  static double yaw_ned_to_enu(double yaw_ned);
  static double yaw_enu_to_ned(double yaw_enu);
  static double wrap_angle(double angle);
};

}  // namespace chr_controller
