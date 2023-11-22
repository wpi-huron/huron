# include "huron_ros2/huron.h"

namespace huron {
namespace ros2 {

Huron::Huron(std::shared_ptr<HuronNode> node,
             std::unique_ptr<huron::Robot::RobotConfiguration> config,
             std::unique_ptr<huron::ZeroMomentPoint> zmp)
  : huron::LeggedRobot(std::move(config), std::move(zmp)),
    node_(std::move(node)) {}

Huron::Huron(std::shared_ptr<HuronNode> node,
             std::unique_ptr<huron::ZeroMomentPoint> zmp)
  : Huron(std::move(node),
          std::make_unique<huron::Robot::RobotConfiguration>(),
          std::move(zmp)) {}

void Huron::Initialize() {
}

void Huron::SetUp() {
}

void Huron::Terminate() {
}

bool Huron::Move(const std::vector<double>& values) {
  node_->PublishJointEffort(values);
  return true;
}

bool Huron::Stop() {
  return Move(std::vector<double>(12));
}

std::vector<double> Huron::GetJointPosition() {
  return node_->joint_position_;
}

std::vector<double> Huron::GetJointVelocity() {
  return node_->joint_velocity_;
}

std::vector<double> Huron::GetForceResistorSensorLeft() {
  return node_->fsr_left_;
}
std::vector<double> Huron::GetForceResistorSensorRight() {
  return node_->fsr_right_;
}
void Huron::Loop() {
  rclcpp::spin_some(node_);
}

}  // namespace ros2
}  // namespace huron
