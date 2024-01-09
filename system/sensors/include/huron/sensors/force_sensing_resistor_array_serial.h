#pragma once

#include <memory>
#include "huron/sensors/force_sensing_resistor_array.h"
#include "huron/driver/serial/serial.h"

namespace huron {

/**
  * An array of FSR with values transmitted over Serial communication.
  *
  * The sensor values are in double but sent in string in the following syntax:
  * <sensor_name>,<val_1>,<val_2>,...,<val_n>\n
  * The sensor values should be sent periodically.
  */
class ForceSensingResistorArraySerial : public ForceSensingResistorArray {
 public:
  ForceSensingResistorArraySerial(
    const std::string& name,
    std::weak_ptr<const multibody::Frame> frame,
    const std::vector<std::shared_ptr<ForceSensingResistor>>& fsr_array,
    std::shared_ptr<driver::serial::SerialBase> serial);
  ForceSensingResistorArraySerial(
    const std::string& name,
    std::weak_ptr<const multibody::Frame> frame,
    const std::vector<std::shared_ptr<ForceSensingResistor>>& fsr_array,
    std::shared_ptr<driver::serial::SerialBase> serial,
    std::unique_ptr<Configuration> config);

  ForceSensingResistorArraySerial(
    const ForceSensingResistorArraySerial&) = delete;

  ForceSensingResistorArraySerial&
    operator=(const ForceSensingResistorArraySerial&) = delete;

  ~ForceSensingResistorArraySerial() override = default;

  void RequestStateUpdate() override;

  Eigen::VectorXd GetValue() const override;
  Eigen::VectorXd ReloadAndGetValue() override;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

 private:
  static inline const std::string delimiter = ",";
  std::shared_ptr<driver::serial::SerialBase> serial_;
};

}  // namespace huron
