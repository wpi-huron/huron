#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

HuronNode::HuronNode()
  : Node("huron_node"),
    joint_state_(Eigen::VectorXd::Zero(kNumPositions + kNumVelocities)),
    wrenches_({huron::Vector6d::Zero(), huron::Vector6d::Zero()}) {
  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(
        &HuronNode::JointStatesCallback,
        this,
        std::placeholders::_1));
  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/p3d/odom",
      10,
      std::bind(
        &HuronNode::OdomCallback,
        this,
        std::placeholders::_1));
  left_ft_sensor_sub_ =
    this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "huron/sensor/l1_ft_sensor",
      10,
      std::bind(
        &HuronNode::LeftFtSensorCallback,
        this,
        std::placeholders::_1));
  right_ft_sensor_sub_ =
    this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "huron/sensor/r1_ft_sensor",
      10,
      std::bind(
        &HuronNode::RightFtSensorCallback,
        this,
        std::placeholders::_1));
  joint_effort_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "joint_group_effort_controller/commands", 10);
}

void HuronNode::JointStatesCallback(
  std::shared_ptr<const sensor_msgs::msg::JointState> msg) {
  for (size_t i = 0; i < msg->position.size(); ++i) {
    // 7 first elements are the floating base
    joint_state_(i + 7) = msg->position[i];
  }
  for (size_t j = 0; j < msg->velocity.size(); ++j) {
    // 6 first elements are the floating base
    joint_state_(j + 6 + kNumPositions) = msg->velocity[j];
  }
}

void HuronNode::OdomCallback(
  std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
  joint_state_.segment(0, 7) << msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w;
  joint_state_.segment(kNumPositions, 6) << msg->twist.twist.linear.x,
                                            msg->twist.twist.linear.y,
                                            msg->twist.twist.linear.z,
                                            msg->twist.twist.angular.x,
                                            msg->twist.twist.angular.y,
                                            msg->twist.twist.angular.z;
}

void HuronNode::LeftFtSensorCallback(
  std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg) {
  wrenches_[0] << msg->wrench.force.x,
                  msg->wrench.force.y,
                  msg->wrench.force.z,
                  msg->wrench.torque.x,
                  msg->wrench.torque.y,
                  msg->wrench.torque.z;
}

void HuronNode::RightFtSensorCallback(
  std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg) {
  wrenches_[1] << msg->wrench.force.x,
                  msg->wrench.force.y,
                  msg->wrench.force.z,
                  msg->wrench.torque.x,
                  msg->wrench.torque.y,
                  msg->wrench.torque.z;
}

void HuronNode::PublishJointEffort(const std::vector<double>& values) {
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  msg.layout.dim[0].size = values.size();
  msg.layout.dim[0].stride = 1;

  msg.data.insert(msg.data.end(), values.begin(), values.end());
  joint_effort_pub_->publish(msg);
}

}  // namespace ros2
}  // namespace huron
