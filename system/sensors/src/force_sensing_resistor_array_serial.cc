#include "huron/sensors/force_sensing_resistor_array_serial.h"
#include "huron/utils/string.h"

namespace huron {

ForceSensingResistorArraySerial::ForceSensingResistorArraySerial(
  const std::string& name,
  std::weak_ptr<const multibody::Frame> frame,
  const std::vector<std::shared_ptr<ForceSensingResistor>>& fsr_array,
  std::shared_ptr<driver::serial::SerialBase> serial)
  : ForceSensingResistorArray(name, std::move(frame), fsr_array),
    serial_(std::move(serial)) {}

ForceSensingResistorArraySerial::ForceSensingResistorArraySerial(
  const std::string& name,
  std::weak_ptr<const multibody::Frame> frame,
  const std::vector<std::shared_ptr<ForceSensingResistor>>& fsr_array,
  std::shared_ptr<driver::serial::SerialBase> serial,
  std::unique_ptr<Configuration> config)
  : ForceSensingResistorArray(name, std::move(frame),
                              fsr_array, std::move(config)),
    serial_(std::move(serial)) {}

void ForceSensingResistorArraySerial::RequestStateUpdate() {
  std::string msg;
  serial_->ReadLine(msg);
  std::vector<std::string> str_values = utils::split(msg, delimiter);
  if (str_values[0] == name_) {
    for (size_t i = 1; i < str_values.size(); ++i) {
      values_[i-1] = std::stod(str_values[i]);
    }
  }
}

Eigen::VectorXd ForceSensingResistorArraySerial::GetValue() const {
  return values_;
}

Eigen::VectorXd ForceSensingResistorArraySerial::ReloadAndGetValue() {
  RequestStateUpdate();
  return values_;
}

void ForceSensingResistorArraySerial::Initialize() {
  serial_->Open();
}

void ForceSensingResistorArraySerial::SetUp() {
}

void ForceSensingResistorArraySerial::Terminate() {
  serial_->Close();
}

}  // namespace huron
