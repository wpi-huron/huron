#pragma once

#include <vector>
#include <set>
#include <string>
#include <utility>
#include <memory>

#include "huron/control_interfaces/actuator.h"

namespace huron {

class MotorConfiguration : public ActuatorConfiguration {
 private:
  static const inline std::set<std::string> kMotorValidKeys{};

 public:
  MotorConfiguration(ConfigMap config_map,
                     std::set<std::string> valid_keys)
      : ActuatorConfiguration(config_map, [&valid_keys]() {
                        std::set<std::string> tmp(kMotorValidKeys);
                        tmp.merge(valid_keys);
                        return tmp;
                      }()) {}

  explicit MotorConfiguration(ConfigMap config_map)
      : MotorConfiguration(config_map, {}) {}

  MotorConfiguration()
      : MotorConfiguration({}, {}) {}
};

class Motor : public Actuator {
 public:
  explicit Motor(std::unique_ptr<MotorConfiguration> config,
                 double gear_ratio = 1.0)
    : Actuator(1, std::move(config)) {}
  explicit Motor(double gear_ratio)
    : Motor(std::make_unique<MotorConfiguration>(), gear_ratio) {}
  Motor() : Motor(1.0) {}
  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;
  ~Motor() override = default;

  virtual bool Move(double value) = 0;

 private:
  double gear_ratio_;
};

}  // namespace huron
