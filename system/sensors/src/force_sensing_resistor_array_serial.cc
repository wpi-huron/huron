#include "huron/sensors/force_sensing_resistor_array_serial.h"
#include "huron/utils/string.h"

namespace huron {

ForceSensingResistorArraySerial::ForceSensingResistorArraySerial(
  std::string name,
  size_t num_sensors,
  std::unique_ptr<ForceSensingResistorArrayConfiguration> config,
  std::shared_ptr<driver::serial::SerialBase> serial)
  : ForceSensingResistorArray(name, num_sensors, std::move(config)),
    serial_(std::move(serial)) {}

ForceSensingResistorArraySerial::ForceSensingResistorArraySerial(
  std::string name,
  size_t num_sensors,
  std::shared_ptr<driver::serial::SerialBase> serial)
  : ForceSensingResistorArraySerial(
      name,
      num_sensors,
      std::make_unique<ForceSensingResistorArrayConfiguration>(),
      std::move(serial)) {}

Eigen::VectorXd ForceSensingResistorArraySerial::GetValues() {
  std::string msg;
  serial_->ReadLine(msg);
  std::vector<std::string> str_values = utils::split(msg, delimiter);
  if (str_values[0] == name_) {
    for (size_t i = 1; i < str_values.size(); ++i) {
      values_[i] = std::stod(str_values[i]);
    }
  }
  return values_eigen_;
}

void ForceSensingResistorArraySerial::Initialize() {
  serial_->Open();
}

void ForceSensingResistorArraySerial::SetUp() {
}

void ForceSensingResistorArraySerial::Terminate() {
}

}  // namespace huron
