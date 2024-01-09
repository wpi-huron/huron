#pragma once

#include <eigen3/Eigen/Core>

#include "robot.h"
#include "huron/locomotion/zero_moment_point.h"

namespace huron {

class LeggedRobot : public Robot {
 public:
  explicit LeggedRobot(std::unique_ptr<RobotConfiguration> config);
  LeggedRobot();
  LeggedRobot(const LeggedRobot&) = delete;
  LeggedRobot& operator=(const LeggedRobot&) = delete;
  ~LeggedRobot() override = default;

  void InitializeZmp(std::shared_ptr<ZeroMomentPoint> zmp);
  /**
   * Computes the Center of Mass in Base frame.
   */
  Eigen::Vector2d EvalZeroMomentPoint();

 private:
  std::shared_ptr<ZeroMomentPoint> zmp_;
};

}  // namespace huron
