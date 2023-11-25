#include "huron/locomotion/zero_moment_point.h"
#include "huron/sensors/force_sensing_resistor_array.h"
#include "huron/math/rotation.h"
#include "huron/types.h"

namespace huron {

ZeroMomentPoint::ZeroMomentPoint(std::string frame,
                                 double normal_force_threshold)
  : frame_(frame),
    normal_force_threshold_(normal_force_threshold) {
}

ZeroMomentPointFTSensor::ZeroMomentPointFTSensor(
  std::string frame,
  double normal_force_threshold,
  const Eigen::Vector3d& sensor_position,
  const Eigen::Vector3d& sensor_frame_zyx,
  std::shared_ptr<ForceTorqueSensor> ft_sensor) 
  : ZeroMomentPoint(frame, normal_force_threshold),
    sensor_position_(sensor_position),
    sensor_frame_rotation_(  // constructs 6x6 rotation matrix
      [&sensor_frame_zyx]() {
        Eigen::Matrix<double, 6, 6> ret;
        Eigen::Matrix3d rot = math::ZyxToRotationMatrix(sensor_frame_zyx);
        ret.topLeftCorner(3, 3) = rot;
        ret.bottomRightCorner(3, 3) = rot;
        return ret;
      }()),
    ft_sensor_(std::move(ft_sensor)) {
}

void ZeroMomentPointFTSensor::Compute(
  Eigen::Ref<Eigen::Vector2d> zmp, double& fz) {
  Vector6d w = sensor_frame_rotation_ * ft_sensor_->GetWrench();
  fz = w(2);
  if (std::abs(w(2)) < normal_force_threshold_) {
    zmp.setZero();
  } else {
    zmp(0) = (-w(4) - w(0)*sensor_position_[2] + sensor_position_[0]*w(2))
      / w(2);
    zmp(1) = (w(3) - w(1)*sensor_position_[2] + sensor_position_[1]*w(2))
      / w(2);
  }
}

ZeroMomentPointFSRArray::ZeroMomentPointFSRArray(
  std::string frame,
  double normal_force_threshold,
  const Eigen::VectorXd& sensor_x_positions,
  const Eigen::VectorXd& sensor_y_positions,
  std::shared_ptr<ForceSensingResistorArray> fsr_array)
  : ZeroMomentPoint(frame, normal_force_threshold),
    sensor_x_positions_(sensor_x_positions),
    sensor_y_positions_(sensor_y_positions),
    fsr_array_(fsr_array) {
}

void ZeroMomentPointFSRArray::Compute(Eigen::Ref<Eigen::Vector2d> zmp,
                                      double& fz) {
  Eigen::VectorXd fz_array = fsr_array_->GetValues().transpose();
  double sum_fz = fz_array.colwise().sum().value();
  fz = sum_fz;
  if (std::abs(sum_fz) < normal_force_threshold_) {
    zmp.setZero();
  } else {
    zmp(0) = (fz_array * sensor_x_positions_).value() / sum_fz;
    zmp(1) = (fz_array * sensor_y_positions_).value() / sum_fz;
  }
}

ZeroMomentPointTotal::ZeroMomentPointTotal(
  std::string frame,
  std::vector<std::shared_ptr<ZeroMomentPoint>> zmp_vector)
  : ZeroMomentPoint(frame, 0),
    zmp_vector_(zmp_vector) {
}

void ZeroMomentPointTotal::Compute(Eigen::Ref<Eigen::Vector2d> zmp,
                                   double& fz) {
  double num_x = 0.0, num_y = 0.0, den = 0.0;
  for (auto& zmp_obj : zmp_vector_) {
    Eigen::Vector2d zmp_i;
    double fz_i;
    zmp_obj->Compute(zmp_i, fz_i);
    num_x += zmp_i(0) * fz_i;
    num_y += zmp_i(1) * fz_i;
    den += fz_i;
  }
  fz = den;
  if (den == 0.0) {
    zmp.setZero();
  } else {
    zmp(0) = num_x / den;
    zmp(1) = num_y / den;
  }
}
    
}  // namespace huron
