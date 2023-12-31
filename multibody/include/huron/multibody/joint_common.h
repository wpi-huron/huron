#pragma once

#include <stddef.h>
#include <string>
#include <cassert>
#include <eigen3/Eigen/Dense>

#include "frame.h"

namespace huron {
namespace multibody {

using JointIndex = size_t;

enum class JointType {
  kUnknown = 0,
  kFixed,
  kRevolute,
  kPrismatic,
  kPlanar,
  kSpherical,
  kFreeFlyer
};

struct JointDescription {
 public:
  // TODO(dtbpkmte): Properly implement JointIndex/FrameIndex and casts.
  JointDescription(size_t id, const std::string& name,
                   size_t parent_frame_id,
                   size_t child_frame_id,
                   size_t num_positions, size_t num_velocities,
                   JointType type,
                   const Eigen::VectorXd& min_position,
                   const Eigen::VectorXd& max_position,
                   const Eigen::VectorXd& min_velocity,
                   const Eigen::VectorXd& max_velocity,
                   const Eigen::VectorXd& min_acceleration,
                   const Eigen::VectorXd& max_acceleration,
                   const Eigen::VectorXd& min_torque,
                   const Eigen::VectorXd& max_torque,
                   const Eigen::VectorXd& friction,
                   const Eigen::VectorXd& damping)
      : id_((JointIndex) id),
        name_(name),
        parent_frame_id_((FrameIndex) parent_frame_id),
        child_frame_id_((FrameIndex) child_frame_id),
        num_positions_(num_positions),
        num_velocities_(num_velocities),
        type_(type),
        min_position_(min_position),
        max_position_(max_position),
        min_velocity_(min_velocity),
        max_velocity_(max_velocity),
        min_acceleration_(min_acceleration),
        max_acceleration_(max_acceleration),
        min_torque_(min_torque),
        max_torque_(max_torque),
        friction_(friction),
        damping_(damping) {
    assert(min_position.size() == num_positions_);
    assert(max_position.size() == num_positions_);
    assert((max_position.array() >= min_position.array()).all());

    assert(min_velocity.size() == num_velocities_);
    assert(max_velocity.size() == num_velocities_);
    assert((max_velocity.array() >= min_velocity.array()).all());

    assert(min_acceleration.size() == num_velocities_);
    assert(max_acceleration.size() == num_velocities_);
    assert((max_acceleration.array() >= min_acceleration.array()).all());

    assert(min_torque.size() == num_velocities_);
    assert(max_torque.size() == num_velocities_);
    assert((max_torque.array() >= min_torque.array()).all());

    assert(friction.size() == num_velocities_);
    assert(damping.size() == num_velocities_);
  }

  JointDescription(size_t id, const std::string& name,
                   size_t parent_frame_id,
                   size_t child_frame_id,
                   size_t num_positions, size_t num_velocities,
                   JointType type,
                   const Eigen::VectorXd& min_position,
                   const Eigen::VectorXd& max_position,
                   const Eigen::VectorXd& min_velocity,
                   const Eigen::VectorXd& max_velocity,
                   const Eigen::VectorXd& min_acceleration,
                   const Eigen::VectorXd& max_acceleration,
                   const Eigen::VectorXd& min_torque,
                   const Eigen::VectorXd& max_torque)
      : JointDescription(id, name,
                         parent_frame_id, child_frame_id,
                         num_positions, num_velocities,
                         type,
                         min_position, max_position,
                         min_velocity, max_velocity,
                         min_acceleration, max_acceleration,
                         min_torque, max_torque,
                         Eigen::VectorXd::Zero(num_velocities),
                         Eigen::VectorXd::Zero(num_velocities)) {}

  JointDescription(size_t id, const std::string& name,
                   size_t parent_frame_id,
                   size_t child_frame_id,
                   size_t num_positions,
                   size_t num_velocities,
                   JointType type)
      : JointDescription(
          id, name,
          parent_frame_id, child_frame_id,
          num_positions, num_velocities,
          type,
          Eigen::VectorXd::Constant(num_positions,
            -std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_positions,
            std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_velocities,
            -std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_velocities,
            std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_velocities,
            -std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_velocities,
            std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_velocities,
            -std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(num_velocities,
            std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Zero(num_velocities),
          Eigen::VectorXd::Zero(num_velocities)) {}

  JointIndex id() const { return id_; }
  const std::string& name() const { return name_; }
  FrameIndex parent_frame_id() const { return parent_frame_id_; }
  FrameIndex child_frame_id() const { return child_frame_id_; }
  size_t num_positions() const { return num_positions_; }
  size_t num_velocities() const { return num_velocities_; }
  JointType type() const { return type_; }
  const Eigen::VectorXd& min_position() const { return min_position_; }
  const Eigen::VectorXd& max_position() const { return max_position_; }
  const Eigen::VectorXd& min_velocity() const { return min_velocity_; }
  const Eigen::VectorXd& max_velocity() const { return max_velocity_; }
  const Eigen::VectorXd& min_acceleration() const { return min_acceleration_; }
  const Eigen::VectorXd& max_acceleration() const { return max_acceleration_; }
  const Eigen::VectorXd& min_torque() const { return min_torque_; }
  const Eigen::VectorXd& max_torque() const { return max_torque_; }
  const Eigen::VectorXd& friction() const { return friction_; }
  const Eigen::VectorXd& damping() const { return damping_; }

 private:
  JointIndex id_;
  std::string name_;
  FrameIndex parent_frame_id_;
  FrameIndex child_frame_id_;
  size_t num_positions_;
  size_t num_velocities_;
  JointType type_;
  Eigen::VectorXd min_position_;
  Eigen::VectorXd max_position_;
  Eigen::VectorXd min_velocity_;
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd min_acceleration_;
  Eigen::VectorXd max_acceleration_;
  Eigen::VectorXd min_torque_;
  Eigen::VectorXd max_torque_;
  Eigen::VectorXd friction_;
  Eigen::VectorXd damping_;
};

}  // namespace multibody
}  // namespace huron
