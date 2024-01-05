#include "huron/control_interfaces/robot.h"

namespace huron {

Robot::Robot(std::unique_ptr<RobotConfiguration> config)
  : GenericComponent(std::move(config)), MovingGroup(),
    model_(std::make_shared<multibody::Model>()) {}

Robot::Robot()
  : Robot::Robot(std::make_unique<RobotConfiguration>()) {}

Robot::Robot(std::unique_ptr<RobotConfiguration> config,
             std::shared_ptr<multibody::Model> model)
  : GenericComponent(std::move(config)), MovingGroup(),
    model_(std::make_shared<multibody::Model>()) {}

Robot::Robot(std::shared_ptr<multibody::Model> model)
  : Robot::Robot(std::make_unique<RobotConfiguration>(), std::move(model)) {}

void Robot::RegisterStateProvider(
  std::shared_ptr<StateProvider> state_provider,
  bool is_joint_state_provider) {
  if (!is_joint_state_provider) {
    non_joint_state_providers_.push_back(state_provider);
  }
}

void Robot::UpdateAllStates() {
  for (auto& state_provider : non_joint_state_providers_) {
    state_provider->RequestStateUpdate();
  }
  model_->UpdateJointStates();
}

void Robot::UpdateJointStates() {
  model_->UpdateJointStates();
}

}  // namespace huron
