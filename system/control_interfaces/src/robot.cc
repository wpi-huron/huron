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

  }  // namespace huron
