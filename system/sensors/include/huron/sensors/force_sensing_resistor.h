#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include "huron/control_interfaces/sensor_with_frame.h"

namespace huron {

class ForceSensingResistor : public SensorWithFrame {
 public:
  ForceSensingResistor(std::weak_ptr<const multibody::Frame> frame);
  ForceSensingResistor(std::weak_ptr<const multibody::Frame> frame,
                       std::unique_ptr<Configuration> config);
  ForceSensingResistor(const ForceSensingResistor&) = delete;
  ForceSensingResistor& operator=(const ForceSensingResistor&) = delete;
  virtual ~ForceSensingResistor() = default;
};

}  // namespace huron
