#pragma once

#include <eigen3/Eigen/Core>

#include "huron/types.h"

namespace huron {

class ForceTorqueSensor {
 public:
  virtual Vector6d GetWrench() = 0;
};

}  // namespace huron
