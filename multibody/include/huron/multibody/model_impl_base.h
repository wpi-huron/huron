#pragma once

#include "model_impl_interface.h"

namespace huron {
namespace multibody {
namespace internal {

class ModelImplBase : public ModelImplInterface {
 public:
  ModelImplBase();
  ModelImplBase(const ModelImplBase&) = delete;
  ModelImplBase& operator=(const ModelImplBase&) = delete;
  ~ModelImplBase() override = default;

  void BuildFromUrdf(const std::string& urdf_path) override;

  std::vector<std::string> GetJointNames() const override;
  std::weak_ptr<Joint> GetJoint(const std::string& name) const override;
  std::weak_ptr<Joint> GetJoint(size_t joint_index) const override;

  std::unique_ptr<JointDescription> GetJointDescription(
    JointIndex joint_index) const override;
  std::unique_ptr<JointDescription> GetJointDescriptionFromChildFrame(
    FrameIndex child_frame_index) const override;

  const Eigen::Affine3d&
    GetJointTransformInWorld(size_t joint_index) const override;

  const FrameIndex& GetFrameIndex(const std::string& frame_name) const override;
  const Eigen::Affine3d& GetFrameTransform(FrameIndex from_frame,
                                           FrameIndex to_frame) const override;
  const Eigen::Affine3d&
    GetFrameTransformInWorld(FrameIndex frame) const override;

  const Eigen::VectorXd& GetAccelerations() const override;
  const Eigen::VectorXd& GetTorques() const override;
  const Eigen::MatrixXd& GetMassMatrix() const override;
  const Eigen::MatrixXd& GetCoriolisMatrix() const override;
  const Eigen::VectorXd& GetNonlinearEffects() const override;
  const Eigen::VectorXd& GetGravity() const override;
  const huron::Vector6d& GetSpatialMomentum() const override;
  const huron::Vector6d& GetCentroidalMomentum() const override;
  const huron::Matrix6Xd& GetCentroidalMatrix() const override;

  void ComputeAll(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v) override;

  void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q) override;
  void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v) override;
  void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& a) override;

  bool is_built() const { return is_built_; }

 protected:
  bool is_built_ = false;
};

}  // namespace internal
}  // namespace multibody
}  // namespace huron
