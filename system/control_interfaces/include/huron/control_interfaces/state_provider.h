#pragma once

#include <eigen3/Eigen/Dense>

namespace huron {

class StateProvider {
 public:
  explicit StateProvider(const Eigen::Vector2i& dim)
    : dim_(dim) {}
  StateProvider(int rows, int cols)
    : dim_(rows, cols) {}
  StateProvider(const StateProvider&) = delete;
  StateProvider& operator=(const StateProvider&) = delete;
  virtual ~StateProvider() = default;

  virtual void RequestStateUpdate() = 0;
  virtual void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const = 0;

  const Eigen::Vector2i& dim() const { return dim_; }

 private:
  const Eigen::Vector2i dim_;
};

}  // namespace huron
