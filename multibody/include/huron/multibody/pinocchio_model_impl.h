#pragma once

#include <experimental/propagate_const>

#include "huron/multibody/model_impl_interface.h"
#include "huron/multibody/joint_common.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

class PinocchioModelImpl : public ModelImplInterface {
 public:
  PinocchioModelImpl();
  PinocchioModelImpl(const PinocchioModelImpl&) = delete;
  PinocchioModelImpl& operator=(const PinocchioModelImpl&) = delete;
  ~PinocchioModelImpl() override;

  static bool IsAvailable() { return true; }

  void BuildFromUrdf(const std::string& urdf_path,
                     JointType root_joint_type) override;

  const std::vector<std::string>& GetJointNames() const override;
  std::weak_ptr<Joint> GetJoint(const std::string& name) const override;
  std::weak_ptr<Joint> GetJoint(size_t joint_index) const override;

  JointType GetJointType(size_t joint_index) const override;
  JointIndex GetJointIndex(const std::string& joint_name) const override;

  std::unique_ptr<JointDescription> GetJointDescription(
    JointIndex joint_index) const override;
  std::unique_ptr<JointDescription> GetJointDescription(
    const std::string& joint_name) const override;

  Eigen::Affine3d
  GetJointTransformInWorld(size_t joint_index) const override;

  FrameIndex GetFrameIndex(const std::string& frame_name) const override;
  const std::string& GetFrameName(FrameIndex frame_index) const override;
  FrameType GetFrameType(FrameIndex frame_index) const override;
  Eigen::Affine3d GetFrameTransform(FrameIndex from_frame,
                                    FrameIndex to_frame) const override;
  Eigen::Affine3d
  GetFrameTransformInWorld(FrameIndex frame) const override;

  Eigen::Vector3d GetCenterOfMassPosition() const override;

  Eigen::VectorXd NeutralConfiguration() const override;

  const Eigen::VectorXd& GetAccelerations() const override;
  const Eigen::VectorXd& GetTorques() const override;
  const Eigen::MatrixXd& GetMassMatrix() const override;
  const Eigen::MatrixXd& GetCoriolisMatrix() const override;
  const Eigen::VectorXd& GetNonlinearEffects() const override;
  const Eigen::VectorXd& GetGravity() const override;
  const huron::Vector6d& GetSpatialMomentum() const override;
  huron::Vector6d GetCentroidalMomentum() const override;
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

  bool is_built() const override { return is_built_; }
  size_t num_positions() const override { return num_positions_; }
  size_t num_velocities() const override { return num_velocities_; }
  size_t num_joints() const override { return num_joints_; }
  size_t num_frames() const override { return num_frames_; }

 private:
  struct Impl;
  std::experimental::propagate_const<std::unique_ptr<Impl>> impl_;

  bool is_built_ = false;
  size_t num_positions_ = 0;
  size_t num_velocities_ = 0;
  size_t num_joints_ = 0;
  size_t num_frames_ = 0;
};

}  // namespace internal
}  // namespace multibody
}  // namespace huron
