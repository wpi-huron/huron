#pragma once

#include <eigen3/Eigen/Core>
#include "huron/control_interfaces/generic_component.h"
#include "huron/control_interfaces/state_provider.h"

namespace huron {

class Sensor : public GenericComponent, public StateProvider {
 public:
  Sensor(const Eigen::Vector2i& dim,
         std::unique_ptr<Configuration> config);
  Sensor(const Eigen::Vector2i& dim);
  Sensor(int rows, int cols,
         std::unique_ptr<Configuration> config);
  Sensor(int rows, int cols);
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;
  virtual ~Sensor() = default;

  /**
   * @brief Get the sensor value.
   */
  virtual Eigen::VectorXd GetValue() const;
  virtual Eigen::VectorXd ReloadAndGetValue();
};

}  // namespace huron
