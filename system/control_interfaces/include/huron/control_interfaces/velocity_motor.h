#pragma once

#include <set>
#include <string>
#include <utility>
#include <memory>

#include "huron/control_interfaces/motor.h"

namespace huron {

class VelocityMotorConfiguration : public MotorConfiguration {
 public:
  /**
   * Supports further inheritance.
   */
  VelocityMotorConfiguration(ConfigMap config_map,
                             std::set<std::string> valid_keys)
    : MotorConfiguration(config_map,
                         [&valid_keys]() {
                           std::set<std::string> tmp(kVelocityMotorValidKeys);
                           tmp.merge(valid_keys);
                           return tmp;
                         }()) {}

  VelocityMotorConfiguration()
    : VelocityMotorConfiguration({}, {}) {}

 private:
  static const inline std::set<std::string> kVelocityMotorValidKeys{};
};

class VelocityMotor : public Motor {
 public:
  explicit VelocityMotor(std::unique_ptr<VelocityMotorConfiguration> config,
                       double gear_ratio)
    : Motor(std::move(config), gear_ratio) {}
  VelocityMotor(double gear_ratio)
    : Motor(gear_ratio) {}
  VelocityMotor() : Motor() {}
  VelocityMotor(const VelocityMotor&) = delete;
  VelocityMotor& operator=(const VelocityMotor&) = delete;
  ~VelocityMotor() override = default;
};

}  // namespace huron
