#pragma once

#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "huron/types.h"


namespace huron {
namespace ros2 {

class HuronNode : public rclcpp::Node {
 public:
  // Floating base + 12 revolute joints
  static constexpr size_t kNumPositions = 7 + 12*1;
  static constexpr size_t kNumVelocities = 6 + 12*1;

  HuronNode();
  ~HuronNode() override = default;

  void JointStatesCallback(
    std::shared_ptr<const sensor_msgs::msg::JointState> msg);
  void OdomCallback(
    std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  void LeftFtSensorCallback(
    std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg);
  void RightFtSensorCallback(
    std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg);
  void PublishJointEffort(const std::vector<double>& values);

  const huron::Vector6d& GetWrench(size_t idx) const {
    return wrenches_[idx];
  }

  Eigen::VectorXd GetJointState(size_t id_q, size_t dim_q,
                                size_t id_v, size_t dim_v) const {
    Eigen::VectorXd state(dim_q + dim_v);
    state.segment(0, dim_q) = joint_state_.segment(id_q, dim_q);
    state.segment(dim_q, dim_v) = joint_state_.segment(id_v, dim_v);
    return state;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_effort_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    left_ft_sensor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    right_ft_sensor_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  Eigen::VectorXd joint_state_;
  std::array<huron::Vector6d, 2> wrenches_;
};

}  // namespace ros2
}  // namespace huron
