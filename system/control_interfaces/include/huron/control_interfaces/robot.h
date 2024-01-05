#pragma once

#include <vector>
#include <set>
#include <string>
#include <utility>
#include <memory>

#include "huron/control_interfaces/configuration.h"
#include "huron/control_interfaces/generic_component.h"
#include "huron/control_interfaces/moving_group.h"
#include "huron/control_interfaces/joint.h"
#include "huron/multibody/model.h"
#include "huron/multibody/joint_common.h"

namespace huron {

class RobotConfiguration : public Configuration {
 private:
  static const inline std::set<std::string> kRobotValidKeys{};

 public:
  RobotConfiguration(ConfigMap config_map,
                     std::set<std::string> valid_keys)
      : Configuration(config_map, [&valid_keys]() {
                        std::set<std::string> tmp(kRobotValidKeys);
                        tmp.merge(valid_keys);
                        return tmp;
                      }()) {}

  explicit RobotConfiguration(ConfigMap config_map)
      : RobotConfiguration(config_map, {}) {}

  RobotConfiguration()
      : RobotConfiguration({}, {}) {}
};

class Robot : public MovingGroup, public GenericComponent {
  using Model = multibody::Model;
 public:
  explicit Robot(std::unique_ptr<RobotConfiguration> config);
  Robot();
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  ~Robot() override = default;

  Model* const GetModel() { return model_.get(); }

  void RegisterStateProvider(std::shared_ptr<StateProvider> state_provider,
                             bool is_joint_state_provider = false);

  /**
   * Calls RequestStateUpdate() on all the registered state providers.
   */
  void UpdateAllStates();

  /**
   * Calls RequestStateUpdate() on all the registered state providers for
   * joints.
   */
  void UpdateJointStates();

 protected:
  Robot(std::unique_ptr<RobotConfiguration> config,
        std::shared_ptr<Model> model);
  explicit Robot(std::shared_ptr<Model> model);

 private:
  std::shared_ptr<Model> model_;
  std::vector<std::shared_ptr<StateProvider>> non_joint_state_providers_;
};

}  // namespace huron
