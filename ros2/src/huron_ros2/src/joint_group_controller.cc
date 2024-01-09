#include "huron_ros2/joint_group_controller.h"
#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

JointGroupController::JointGroupController(std::shared_ptr<HuronNode> node)
  : huron::MovingInterface(kNumActuators), node_(std::move(node)) {}

bool JointGroupController::Move(const std::vector<double>& values) {
  node_->PublishJointEffort(values);
  return true;
}

bool JointGroupController::Move(const Eigen::VectorXd& values) {
  node_->PublishJointEffort(
    std::vector<double>(values.data(), values.data() + values.size()));
  return true;
}

bool JointGroupController::Stop() {
  return Move(std::vector<double>(12));
}


}  // namespace ros2
}  // namespace huron
