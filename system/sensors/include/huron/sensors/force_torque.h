#pragma once

#include <eigen3/Eigen/Core>

#include "huron/types.h"

namespace huron {

class ForceTorqueSensor {
 public:
  /**
   * Measures the external forces and moments.
   *
   * @return Wrench 6x1 vector \f$ [Fx, Fy, Fz, Tx, Ty, Tz]^T \f$.
   */
  virtual Vector6d GetWrench() = 0;
};

}  // namespace huron
