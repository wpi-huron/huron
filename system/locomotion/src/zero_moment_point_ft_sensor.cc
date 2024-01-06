#include "huron/locomotion/zero_moment_point_ft_sensor.h"

namespace huron {

ZeroMomentPointFTSensor::ZeroMomentPointFTSensor(
  std::weak_ptr<const multibody::Frame> zmp_frame,
  double normal_force_threshold,
  const std::vector<std::shared_ptr<ForceTorqueSensor>>& ft_sensors)
  : ZeroMomentPoint(std::move(zmp_frame), normal_force_threshold),
    ft_sensors_(ft_sensors) {
}

// Generate Doxygen documentation for the following function
Eigen::Vector2d ZeroMomentPointFTSensor::Eval(double& fz) {
  Eigen::Vector2d zmp;
  double num_x = 0.0, num_y = 0.0, den = 0.0;
  for (auto& ft_sensor : ft_sensors_) {
    Eigen::Affine3d zmp_to_sensor = zmp_frame_.lock()->GetTransformToFrame(
      *ft_sensor->GetSensorFrame().lock());
    Eigen::Affine3d zmp_frame_pose = zmp_frame_.lock()->GetTransformInWorld();
    Vector6d w = ft_sensor->GetValue();
    w.segment(0, 3) = zmp_to_sensor.rotation() * w.segment(0, 3);
    w.segment(3, 3) = zmp_to_sensor.rotation() * w.segment(3, 3);
    num_x +=
      (-w(4) -
      w(0)*(zmp_to_sensor.translation().z() - zmp_frame_pose.translation().z())
      + zmp_to_sensor.translation().x()*w(2));
    num_y += 
      (w(3) -
      w(1)*(zmp_to_sensor.translation().z() - zmp_frame_pose.translation().z())
      + zmp_to_sensor.translation().y()*w(2));
    den += w(2);
  }
  if (std::abs(den) < normal_force_threshold_) {
    zmp.setZero();
  } else {
    zmp.x() = num_x / den;
    zmp.y() = num_y / den;
  }
  fz = den;
  return zmp;
}

}  // namespace huron
