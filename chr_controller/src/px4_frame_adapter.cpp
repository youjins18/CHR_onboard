#include "chr_controller/px4_frame_adapter.hpp"

#include <cmath>

namespace chr_controller {
namespace {
Eigen::Matrix3d enu_from_ned() {
  Eigen::Matrix3d matrix;
  matrix << 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0;
  return matrix;
}

Eigen::Matrix3d frd_from_flu() {
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  matrix(1, 1) = -1.0;
  matrix(2, 2) = -1.0;
  return matrix;
}
}  // namespace

Eigen::Vector3d Px4FrameAdapter::ned_to_enu(const Eigen::Vector3d &ned) {
  return enu_from_ned() * ned;
}

Eigen::Vector3d Px4FrameAdapter::enu_to_ned(const Eigen::Vector3d &enu) {
  return enu_from_ned().transpose() * enu;
}

Eigen::Quaterniond Px4FrameAdapter::ned_frd_to_enu_flu(
    const Eigen::Quaterniond &ned_from_frd) {
  const Eigen::Matrix3d rotation =
    enu_from_ned() * ned_from_frd.normalized().toRotationMatrix() * frd_from_flu();
  return Eigen::Quaterniond(rotation).normalized();
}

Eigen::Quaterniond Px4FrameAdapter::enu_flu_to_ned_frd(
    const Eigen::Quaterniond &enu_from_flu) {
  const Eigen::Matrix3d rotation =
    enu_from_ned().transpose() * enu_from_flu.normalized().toRotationMatrix() *
    frd_from_flu().transpose();
  return Eigen::Quaterniond(rotation).normalized();
}

double Px4FrameAdapter::yaw_ned_to_enu(double yaw_ned) {
  return wrap_angle(0.5 * 3.14159265358979323846 - yaw_ned);
}

double Px4FrameAdapter::yaw_enu_to_ned(double yaw_enu) {
  return wrap_angle(0.5 * 3.14159265358979323846 - yaw_enu);
}

double Px4FrameAdapter::wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace chr_controller
