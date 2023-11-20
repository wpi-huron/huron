#pragma once

#include <eigen3/Eigen/Core>

#include "huron/control_interfaces/generic_component.h"

namespace huron {

class ForceSensingResistorArrayConfiguration : public Configuration {
 private:
  static const inline std::set<std::string> kFsrArrayValidKeys{};

 public:
  ForceSensingResistorArrayConfiguration(
    ConfigMap config_map,
    std::set<std::string> valid_keys)
      : Configuration(config_map, [&valid_keys]() {
                        std::set<std::string> tmp(kFsrArrayValidKeys);
                        tmp.merge(valid_keys);
                        return tmp;
                      }()) {}

  explicit ForceSensingResistorArrayConfiguration(ConfigMap config_map)
      : ForceSensingResistorArrayConfiguration(config_map, {}) {}

  ForceSensingResistorArrayConfiguration()
      : ForceSensingResistorArrayConfiguration({}, {}) {}
};

class ForceSensingResistorArray : public GenericComponent{
 public:
  ForceSensingResistorArray(
    std::string name,
    size_t num_sensors,
    std::unique_ptr<ForceSensingResistorArrayConfiguration> config)
    : GenericComponent(std::move(config)),
      name_(name),
      num_sensors_(num_sensors),
      values_(std::vector<double>(num_sensors)),
      values_eigen_(values_.data(), num_sensors, 1) {}

  explicit ForceSensingResistorArray(std::string name, size_t num_sensors)
    : ForceSensingResistorArray(
        name,
        num_sensors,
        std::make_unique<ForceSensingResistorArrayConfiguration>()) {}

  ForceSensingResistorArray(const ForceSensingResistorArray&) = delete;

  ForceSensingResistorArray&
    operator=(const ForceSensingResistorArray&) = delete;

  virtual ~ForceSensingResistorArray() = default;

  virtual Eigen::VectorXd GetValues() = 0;

 protected:
  std::string name_;
  size_t num_sensors_;
  std::vector<double> values_;
  Eigen::Map<Eigen::VectorXd> values_eigen_;
};

}  // namespace huron
