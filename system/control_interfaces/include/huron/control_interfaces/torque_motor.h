#pragma once

#include <set>
#include <string>
#include <utility>
#include <memory>

#include "huron/control_interfaces/motor.h"

namespace huron {

class TorqueMotorConfiguration : public MotorConfiguration {
 public:
  /**
   * Supports further inheritance.
   */
  TorqueMotorConfiguration(ConfigMap config_map,
                             std::set<std::string> valid_keys)
    : MotorConfiguration(config_map,
                         [&valid_keys]() {
                           std::set<std::string> tmp(kTorqueMotorValidKeys);
                           tmp.merge(valid_keys);
                           return tmp;
                         }()) {}

  TorqueMotorConfiguration()
    : TorqueMotorConfiguration({}, {}) {}

 private:
  static const inline std::set<std::string> kTorqueMotorValidKeys{};
};

class TorqueMotor : public Motor {
 public:
  explicit TorqueMotor(std::unique_ptr<TorqueMotorConfiguration> config,
                       double gear_ratio)
    : Motor(std::move(config), gear_ratio) {}
  TorqueMotor(double gear_ratio)
    : Motor(gear_ratio) {}
  TorqueMotor() : Motor() {}
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  ~TorqueMotor() override = default;
};

}  // namespace huron
