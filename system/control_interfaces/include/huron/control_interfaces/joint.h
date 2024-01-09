#pragma once

#include <memory>
#include <vector>
#include <set>
#include <string>

#include "huron/multibody/joint_common.h"
#include "huron/control_interfaces/configuration.h"
#include "huron/control_interfaces/state_provider.h"

namespace huron {

class Joint {
  using JointDescription = huron::multibody::JointDescription;
  using JointType = huron::multibody::JointType;

 public:
  /**
   * Creates a Joint that connects the specfied parent and child frames.
   */
  explicit Joint(std::unique_ptr<JointDescription> joint_desc,
                 std::shared_ptr<StateProvider> state_provider = nullptr);
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;
  virtual ~Joint() = default;

  void SetIndices(size_t id_q, size_t id_v) {
    id_q_ = id_q;
    id_v_ = id_v;
  }

  /**
   * @brief Sets the state provider for this joint.
   * @note The state provider must return a VectorXd with the correct format
   * and size:
   * \[ \begin{bmatrix} q \\ \dot{q} \end{bmatrix} \in \mathbb{R}^{nq + nv}\]
   */
  void SetStateProvider(std::shared_ptr<StateProvider> state_provider);

  /**
   * @brief Updates the joint state from the state provider.
   */
  void UpdateState();

  JointType GetJointType() const {
    return jd_->type();
  }

  const Eigen::VectorXd& GetPositions() const {
    return positions_;
  }

  const Eigen::VectorXd& GetVelocities() const {
    return velocities_;
  }

  const JointDescription* const Info() const {
    return jd_.get();
  }

  bool IsFullyConfigured() const {
    return jd_->type() == JointType::kUnknown || state_provider_ != nullptr;
  }

  size_t id_q() const {
    return id_q_;
  }

  size_t id_v() const {
    return id_v_;
  }

  size_t num_positions() const {
    return jd_->num_positions();
  }
  
  size_t nq() const {
    return num_positions();
  }

  size_t num_velocities() const {
    return jd_->num_velocities();
  }

  size_t nv() const {
    return num_velocities();
  }

 protected:
  std::unique_ptr<JointDescription> jd_;
  Eigen::VectorXd positions_;
  Eigen::VectorXd velocities_;
  size_t id_q_;
  size_t id_v_;

  std::shared_ptr<StateProvider> state_provider_;
};

}  // namespace huron
