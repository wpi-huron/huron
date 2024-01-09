#include "huron/control_interfaces/joint.h"

namespace huron {

Joint::Joint(std::unique_ptr<JointDescription> joint_desc,
             std::shared_ptr<StateProvider> state_provider)
  : jd_(std::move(joint_desc)),
    positions_(Eigen::VectorXd::Zero(jd_->num_positions())),
    velocities_(Eigen::VectorXd::Zero(jd_->num_velocities())),
    state_provider_(std::move(state_provider)) {}

void Joint::SetStateProvider(std::shared_ptr<StateProvider> state_provider) {
  assert(
    state_provider->dim()[0] == (jd_->num_positions() + jd_->num_velocities())
    &&
    state_provider->dim()[1] == 1);
  state_provider_ = state_provider;
}

void Joint::UpdateState() {
  assert(state_provider_ != nullptr);
  state_provider_->RequestStateUpdate();
  Eigen::VectorXd tmp(jd_->num_positions() + jd_->num_velocities());
  state_provider_->GetNewState(tmp);
  positions_ = tmp.segment(0, jd_->num_positions());
  velocities_ = tmp.segment(jd_->num_positions(), jd_->num_velocities());
}

}  // namespace huron
