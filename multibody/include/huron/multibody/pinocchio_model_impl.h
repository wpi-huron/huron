#pragma once

#include "model_impl_base.h"

#ifdef HURON_USE_PINOCCHIO
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#endif

namespace huron {
namespace multibody {

namespace internal {

template <bool has_pinocchio>
class PinocchioModelImpl final : public ModelImplBase {
 public:
  PinocchioModelImpl();
  PinocchioModelImpl(const PinocchioModelImpl&) = delete;
  PinocchioModelImpl& operator=(const PinocchioModelImpl&) = delete;
  ~PinocchioModelImpl() = default;

  static bool IsAvailable() { return has_pinocchio; }
};

template <>
class PinocchioModelImpl<true> final : public ModelImplBase {
 public:
  PinocchioModelImpl();
  PinocchioModelImpl(const PinocchioModelImpl&) = delete;
  PinocchioModelImpl& operator=(const PinocchioModelImpl&) = delete;
  ~PinocchioModelImpl() override = default;

  static bool IsAvailable() { return true; }

  void BuildFromUrdf(const std::string& urdf_path) override;

  std::vector<std::string> GetJointNames() const override;

  std::weak_ptr<Joint> GetJoint(const std::string& name) const override;
  std::weak_ptr<Joint> GetJoint(size_t joint_index) const override;

  std::unique_ptr<JointDescription> GetJointDescription(
    JointIndex joint_index) const override;

  std::unique_ptr<JointDescription> GetJointDescriptionFromChildFrame(
    FrameIndex child_frame_index) const override;

  Eigen::Affine3d GetJointPoseInWorld(size_t joint_index) const override;


  void GetFrame(const std::string& frame_name) const override;

  /**
   * @brief Get the joint accelerations.
   * @note This method returns the joint accelerations in pinocchio::Data.
   */
  Eigen::VectorXd GetAccelerations() const override;
  Eigen::VectorXd GetTorques() const override;
  Eigen::MatrixXd GetMassMatrix() const override;
  Eigen::MatrixXd GetCoriolisMatrix() const override;
  Eigen::VectorXd GetNonlinearEffects() const override;
  Eigen::VectorXd GetGravity() const override;
  huron::Vector6d GetSpatialMomentum() const override;
  huron::Vector6d GetCentroidalMomentum() const override;
  huron::Matrix6Xd GetCentroidalMatrix() const override;

  void ComputeAll(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v) override;

 protected:
  JointType GetJointType(size_t joint_index) const override;

 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
};

}  // namespace internal

using PinocchioModelImpl = internal::PinocchioModelImpl<HURON_USE_PINOCCHIO>;

}  // namespace multibody
}  // namespace huron
