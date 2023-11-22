#pragma once

#include <eigen3/Eigen/Core>

#include "robot.h"
#include "huron/locomotion/zero_moment_point.h"

namespace huron {

class LeggedRobot : public Robot {
 public:
  LeggedRobot(std::unique_ptr<RobotConfiguration> config,
              std::unique_ptr<ZeroMomentPoint> zmp);
  explicit LeggedRobot(std::unique_ptr<ZeroMomentPoint> zmp);

  Eigen::Vector2d ComputeZeroMomentPoint();

 private:
  std::unique_ptr<ZeroMomentPoint> zmp_;
};

}  // namespace huron
