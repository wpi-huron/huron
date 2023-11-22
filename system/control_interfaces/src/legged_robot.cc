#include "huron/control_interfaces/legged_robot.h"

namespace huron {

LeggedRobot::LeggedRobot(std::unique_ptr<RobotConfiguration> config,
                         std::unique_ptr<ZeroMomentPoint> zmp)
  : Robot(std::move(config)), zmp_(std::move(zmp)) {}

LeggedRobot::LeggedRobot(std::unique_ptr<ZeroMomentPoint> zmp)
  : LeggedRobot(std::make_unique<RobotConfiguration>(), std::move(zmp)) {}

Eigen::Vector2d LeggedRobot::ComputeZeroMomentPoint() {
  return zmp_->Compute();
}

}  // namespace huron
