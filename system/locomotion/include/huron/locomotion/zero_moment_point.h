#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>

#include "huron/sensors/force_torque_sensor.h"
#include "huron/sensors/force_sensing_resistor_array.h"
#include "huron/multibody/logical_frame.h"

namespace huron {

class ZeroMomentPoint {
 public:
  ZeroMomentPoint(std::weak_ptr<const multibody::Frame> zmp_frame,
                  double normal_force_threshold);
  ZeroMomentPoint(const ZeroMomentPoint&) = delete;
  ZeroMomentPoint& operator=(const ZeroMomentPoint&) = delete;
  virtual ~ZeroMomentPoint() = default;

  /**
   * Evaluate the zero moment point in the ZMP frame based on the current
   * sensor and joint states.
   *
   * @param zmp The zero moment point in the ZMP frame.
   * @param fz The normal force.
   */
  virtual Eigen::Vector2d Eval(double& fz) = 0;
  Eigen::Vector2d Eval() {
    double fz;
    return Eval(fz);
  }

  /**
   * Convert the zero moment point from the 2D ZMP frame to the world frame.
   *
   * @param zmp The zero moment point in the ZMP frame.
   * @return The zero moment point in the world frame.
   */
  Eigen::Affine3d ZmpToWorld(const Eigen::Vector2d& zmp) const;

 protected:
  std::weak_ptr<const multibody::Frame> zmp_frame_;
  double normal_force_threshold_;
};

}  // namespace huron
