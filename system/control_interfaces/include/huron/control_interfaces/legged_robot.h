#pragma once

#include <eigen3/Eigen/Core>

#include "robot.h"
#include "huron/locomotion/zero_moment_point.h"
#include "huron/rigid_body/model.h"

namespace huron {

class LeggedRobot : public Robot {
 public:
  LeggedRobot(std::unique_ptr<rigid_body::Model> model,
              std::unique_ptr<RobotConfiguration> config,
              std::unique_ptr<ZeroMomentPoint> zmp);
  explicit LeggedRobot(std::unique_ptr<ZeroMomentPoint> zmp);

  /**
   * Computes the Center of Mass in Base frame.
   */
  Eigen::Affine3d ComputeCenterOfMass();
  Eigen::Affine3d ComputeCenterOfMassInWorldFrame();

  Eigen::Vector2d ComputeZeroMomentPoint();

 private:
  rigid_body::Frame
  std::unique_ptr<ZeroMomentPoint> zmp_;
};

}  // namespace huron
