#pragma once

#include <memory>
#include "force_sensing_resistor_array.h"
#include "huron/driver/serial/serial.h"

namespace huron {

class ForceSensingResistorArraySerial : public ForceSensingResistorArray {
 public:
  ForceSensingResistorArraySerial(
    std::string name,
    size_t num_sensors,
    std::unique_ptr<ForceSensingResistorArrayConfiguration> config,
    std::shared_ptr<driver::serial::SerialBase> serial);

  ForceSensingResistorArraySerial(
    std::string name,
    size_t num_sensors,
    std::shared_ptr<driver::serial::SerialBase> serial);

  ForceSensingResistorArraySerial(
    const ForceSensingResistorArraySerial&) = delete;

  ForceSensingResistorArraySerial&
    operator=(const ForceSensingResistorArraySerial&) = delete;

  virtual ~ForceSensingResistorArraySerial() = default;

  Eigen::VectorXd GetValues() override;

 private:
  static inline const std::string delimiter = ",";
  std::shared_ptr<driver::serial::SerialBase> serial_;
};

}  // namespace huron
