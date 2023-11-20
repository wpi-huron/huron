#pragma once

#include <memory>
#include "force_sensing_resistor_array.h"
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

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

 private:
  static inline const std::string delimiter = ",";
  std::shared_ptr<driver::serial::SerialBase> serial_;
};

}  // namespace huron
