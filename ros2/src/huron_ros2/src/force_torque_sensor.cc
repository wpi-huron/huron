#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

ForceTorqueSensor::ForceTorqueSensor(
  size_t index,
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame> frame,
  std::weak_ptr<const HuronNode> node)
  : huron::ForceTorqueSensor(reverse_wrench_direction, std::move(frame)),
    index_(index),
    node_(std::move(node)) {}

void ForceTorqueSensor::RequestStateUpdate() {
}

void ForceTorqueSensor::Initialize() {

}

void ForceTorqueSensor::SetUp() {

}

void ForceTorqueSensor::Terminate() {

}


Vector6d ForceTorqueSensor::DoGetWrenchRaw() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("HuronNode is expired.");
  }
  return node->GetWrench(index_);
}

}  // namespace ros2
}  // namespace huron
