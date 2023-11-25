#pragma once

#include <eigen3/Eigen/Core>

#include "huron/control_interfaces/generic_component.h"
#include "huron/types.h"

namespace huron {

class ForceTorqueSensorConfiguration : public Configuration {
 public:
  ForceTorqueSensorConfiguration(
    ConfigMap config_map,
    std::set<std::string> valid_keys)
      : Configuration(config_map, [&valid_keys]() {
                        std::set<std::string> tmp(kFtSensorValidKeys);
                        tmp.merge(valid_keys);
                        return tmp;
                      }()) {}

  explicit ForceTorqueSensorConfiguration(ConfigMap config_map)
      : ForceTorqueSensorConfiguration(config_map, {}) {}

  ForceTorqueSensorConfiguration()
      : ForceTorqueSensorConfiguration({}, {}) {}

 private:
  static const inline std::set<std::string> kFtSensorValidKeys{};

};

class ForceTorqueSensor : public GenericComponent {
 public:
  ForceTorqueSensor(
    std::string name,
    bool reverse_wrench_direction,
    std::unique_ptr<ForceTorqueSensorConfiguration> config)
    : GenericComponent(std::move(config)),
      name_(name),
      reverse_wrench_direction_(reverse_wrench_direction) {}

  ForceTorqueSensor(std::string name, bool reverse_wrench_direction = false)
    : ForceTorqueSensor(name,
                        reverse_wrench_direction,
                        std::make_unique<ForceTorqueSensorConfiguration>()) {}
  /**
   * Measures the external forces and moments.
   *
   * @return Wrench 6x1 vector \f$ [Fx, Fy, Fz, Tx, Ty, Tz]^T \f$.
   */
  Vector6d GetWrench() {
    if (reverse_wrench_direction_)
      return -GetWrenchRaw();
    return GetWrenchRaw();
  }
  
 protected:
  /**
   * To be overriden.
   */
  virtual Vector6d GetWrenchRaw() = 0;

  std::string name_;
  bool reverse_wrench_direction_;
};

}  // namespace huron
