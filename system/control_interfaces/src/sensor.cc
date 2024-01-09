#include "huron/control_interfaces/sensor.h"
#include <memory>

namespace huron {

Sensor::Sensor(const Eigen::Vector2i& dim,
               std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)),
      StateProvider(dim) {}

Sensor::Sensor(const Eigen::Vector2i& dim)
  : GenericComponent(),
    StateProvider(dim) {}

Sensor::Sensor(int rows, int cols,
               std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)),
      StateProvider(rows, cols) {}

Sensor::Sensor(int rows, int cols)
  : GenericComponent(),
    StateProvider(rows, cols) {}

Eigen::VectorXd Sensor::GetValue() const {
  Eigen::VectorXd tmp;
  GetNewState(tmp);
  return tmp;
}

Eigen::VectorXd Sensor::ReloadAndGetValue() {
  RequestStateUpdate();
  return GetValue();
}

}  // namespace huron
