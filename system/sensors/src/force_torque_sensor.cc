#include "huron/sensors/force_torque_sensor.h"

namespace huron {

ForceTorqueSensor::ForceTorqueSensor(
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame> frame)
  : SensorWithFrame(6, 1, std::move(frame)),
    reverse_wrench_direction_(reverse_wrench_direction) {}

ForceTorqueSensor::ForceTorqueSensor(
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame> frame,
  std::unique_ptr<Configuration> config)
  : SensorWithFrame(6, 1, std::move(frame), std::move(config)),
    reverse_wrench_direction_(reverse_wrench_direction) {}

void ForceTorqueSensor::RequestStateUpdate() {
  wrench_ = DoGetWrenchRaw();
}

void ForceTorqueSensor::GetNewState(
  Eigen::Ref<Eigen::MatrixXd> new_state) const {
  new_state = GetValue();
}

Eigen::VectorXd ForceTorqueSensor::GetValue() const {
  if (reverse_wrench_direction_)
    return -wrench_;
  return wrench_;
}

}  // namespace huron
