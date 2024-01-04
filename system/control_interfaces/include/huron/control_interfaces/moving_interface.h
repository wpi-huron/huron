#pragma once

#include <vector>
#include <memory>
#include <utility>

#include <eigen3/Eigen/Dense>

namespace huron {

/**
 * Interface for components that can move.
 *
 * @ingroup control_interfaces
 */
class MovingInterface {
 public:
  MovingInterface(size_t dim) : dim_(dim) {}
  MovingInterface(const MovingInterface&) = delete;
  MovingInterface& operator=(const MovingInterface&) = delete;
  virtual ~MovingInterface() = default;

  /**
   * Moves the component by the specified input vector.
   *
   * This method can be used if the component needs more than one input.
   * For example, a position controlled motor needs position input,
   * velocity feedforward, and current feedforward.
   *
   * @param value Input value vector.
   * @return true if the operation is successful, false otherwise.
   */
  virtual bool Move(const std::vector<double>& values) = 0;

  virtual bool Move(const Eigen::VectorXd& values) = 0;

  /**
   * Stops the component from moving.
   *
   * @return true if the operation is successful, false otherwise.
   */
  virtual bool Stop() = 0;

  size_t dim() const { return dim_; }

 protected:
  size_t dim_;
};

}  // namespace huron
