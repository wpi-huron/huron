#include "huron/control_interfaces/legged_robot.h"

namespace huron {

LeggedRobot::LeggedRobot(std::unique_ptr<RobotConfiguration> config)
  : Robot(std::move(config)) {}

LeggedRobot::LeggedRobot()
  : Robot() {}

void LeggedRobot::InitializeZmp(std::shared_ptr<ZeroMomentPoint> zmp) {
  zmp_ = std::move(zmp);
}

Eigen::Vector2d LeggedRobot::EvalZeroMomentPoint() {
  return zmp_->Eval();
}

}  // namespace huron
