#include "huron/sensors/force_sensing_resistor_array.h"

namespace huron {

ForceSensingResistorArray::ForceSensingResistorArray(
  const std::string& name,
  std::weak_ptr<const multibody::Frame> frame,
  const std::vector<std::shared_ptr<ForceSensingResistor>>& fsr_array)
  : SensorWithFrame(fsr_array.size(), 1, std::move(frame)),
    name_(name),
    values_(Eigen::VectorXd::Zero(fsr_array.size())),
    fsr_array_(fsr_array) {}

ForceSensingResistorArray::ForceSensingResistorArray(
  const std::string& name,
  std::weak_ptr<const multibody::Frame> frame,
  const std::vector<std::shared_ptr<ForceSensingResistor>>& fsr_array,
  std::unique_ptr<Configuration> config)
  : SensorWithFrame(fsr_array.size(), 1, std::move(frame), std::move(config)),
    name_(name),
    values_(Eigen::VectorXd::Zero(fsr_array.size())),
    fsr_array_(fsr_array) {}

void ForceSensingResistorArray::RequestStateUpdate() {
  for (size_t i = 0; i < fsr_array_.size(); ++i) {
    values_(i) = fsr_array_[i]->ReloadAndGetValue()(0);
  }
}

void ForceSensingResistorArray::GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const {
  new_state = values_;
}

Eigen::Affine3d ForceSensingResistorArray::GetSensorPose(size_t index) const {
  return fsr_array_[index]->GetSensorFrame().lock()->GetTransformInWorld();
}

}  // namespace huron
