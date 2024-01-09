#include "huron/control_interfaces/moving_group.h"

namespace huron {

MovingGroup::MovingGroup()
  : MovingInterface(0) {}

void MovingGroup::AddToGroup(std::shared_ptr<MovingInterface> component) {
  if (component != nullptr) {
    moving_components_.push_back(std::move(component));
    moving_interface_dims_.push_back(moving_components_.back()->dim());
    dim_ += moving_interface_dims_.back();
    return;
  }
  throw std::runtime_error("Cannot add nullptr to group.");
}

bool MovingGroup::Move(const std::vector<double>& values) {
  if (values.size() != dim_) {
    throw std::runtime_error("Invalid input dimension.");
  }
  size_t offset = 0;
  for (size_t i = 0; i < moving_components_.size(); ++i) {
    std::vector<double> tmp(values.begin() + offset,
                            values.begin() + offset + moving_interface_dims_[i]);
    if (!moving_components_[i]->Move(tmp)) {
      return false;
    }
    offset += moving_interface_dims_[i];
  }
  return true;
}

bool MovingGroup::Move(const Eigen::VectorXd& values) {
  if (values.size() != dim_) {
    throw std::runtime_error("Invalid input dimension.");
  }
  size_t offset = 0;
  for (size_t i = 0; i < moving_components_.size(); ++i) {
    if (!moving_components_[i]->Move(
        values.segment(offset, moving_interface_dims_[i]))) {
      return false;
    }
    offset += moving_interface_dims_[i];
  }
  return true;
}

bool MovingGroup::Stop() {
  for (const auto& component : moving_components_) {
    if (!component->Stop()) {
      return false;
    }
  }
  return true;
}

}  // namespace huron
