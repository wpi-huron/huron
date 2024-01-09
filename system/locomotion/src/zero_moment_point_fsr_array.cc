#include "huron/locomotion/zero_moment_point_fsr_array.h"

namespace huron {

ZeroMomentPointFSRArray::ZeroMomentPointFSRArray(
  std::weak_ptr<const multibody::Frame> zmp_frame,
  double normal_force_threshold,
  std::shared_ptr<ForceSensingResistorArray> fsr_array)
  : ZeroMomentPoint(std::move(zmp_frame), normal_force_threshold),
    fsr_array_(std::move(fsr_array)) {
}

Eigen::Vector2d ZeroMomentPointFSRArray::Eval(double& fz) {
  Eigen::Vector2d zmp;
  Eigen::VectorXd fz_array = fsr_array_->GetValue();
  double sum_fz = fz_array.colwise().sum().value();
  fz = sum_fz;
  if (std::abs(sum_fz) < normal_force_threshold_) {
    zmp.setZero();
  } else {
    double num_x = 0.0, num_y = 0.0;
    for (size_t i = 0; i < fsr_array_->num_sensors(); ++i) {
      num_x += fz_array(i) * fsr_array_->GetSensorPose(i).translation().x();
      num_y += fz_array(i) * fsr_array_->GetSensorPose(i).translation().y();
    }
    zmp(0) = num_x / sum_fz;
    zmp(1) = num_y / sum_fz;
  }
  return zmp;
}

}  // namespace huron
