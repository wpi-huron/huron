#pragma once

#include "huron/control_interfaces/moving_interface.h"

namespace huron {
namespace ros2 {

class HuronNode;

class JointGroupController : public huron::MovingInterface {
 public:
  static constexpr size_t kNumActuators = 12;

  explicit JointGroupController(std::shared_ptr<HuronNode> node);
  JointGroupController(const JointGroupController&) = delete;
  JointGroupController& operator=(const JointGroupController&) = delete;
  ~JointGroupController() override = default;

  bool Move(const std::vector<double>& values) override;
  bool Move(const Eigen::VectorXd& values) override;
  bool Stop() override;

 private:
  std::shared_ptr<HuronNode> node_;
};

}  // namespace ros2
}  // namespace huron
