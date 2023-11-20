#pragma once

#include <eigen3/Eigen/Core>

namespace huron {

class ForceSensingResistor {
 public:
  virtual double GetValue() = 0;
};

}  // namespace huron
