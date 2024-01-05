#include "huron/multibody/model_impl_interface.h"

namespace huron {
namespace multibody {
namespace internal {

void ModelImplInterface::BuildFromUrdf(const std::string& urdf_path,
                                       JointType root_joint_type) {
  throw NotImplementedException();
}

const std::vector<std::string>& ModelImplInterface::GetJointNames() const {
  throw NotImplementedException();
}
std::weak_ptr<Joint> ModelImplInterface::GetJoint(const std::string& name) const {
  throw NotImplementedException();
}
std::weak_ptr<Joint> ModelImplInterface::GetJoint(size_t joint_index) const {
  throw NotImplementedException();
}

JointType ModelImplInterface::GetJointType(size_t joint_index) const {
  throw NotImplementedException();
}

JointIndex
ModelImplInterface::GetJointIndex(const std::string& joint_name) const {
  throw NotImplementedException();
}

std::unique_ptr<JointDescription> ModelImplInterface::GetJointDescription(
  JointIndex joint_index) const {
  throw NotImplementedException();
}
std::unique_ptr<JointDescription> ModelImplInterface::GetJointDescription(
  const std::string& joint_name) const {
  throw NotImplementedException();
}

Eigen::Affine3d
ModelImplInterface::GetJointTransformInWorld(size_t joint_index) const {
  throw NotImplementedException();
}

FrameIndex ModelImplInterface::GetFrameIndex(
  const std::string& frame_name) const {
  throw NotImplementedException();
}
const std::string& ModelImplInterface::GetFrameName(FrameIndex frame_index) const {
  throw NotImplementedException();
}
FrameType ModelImplInterface::GetFrameType(FrameIndex frame_index) const {
  throw NotImplementedException();
}
Eigen::Affine3d ModelImplInterface::GetFrameTransform(FrameIndex from_frame,
                                          FrameIndex to_frame) const {
  throw NotImplementedException();
}
Eigen::Affine3d ModelImplInterface::GetFrameTransformInWorld(FrameIndex frame) const {
  throw NotImplementedException();
}

Eigen::Vector3d ModelImplInterface::GetCenterOfMassPosition() const {
  throw NotImplementedException();
}

Eigen::VectorXd ModelImplInterface::NeutralConfiguration() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplInterface::GetAccelerations() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplInterface::GetTorques() const {
  throw NotImplementedException();
}

const Eigen::MatrixXd& ModelImplInterface::GetMassMatrix() const {
  throw NotImplementedException();
}

const Eigen::MatrixXd& ModelImplInterface::GetCoriolisMatrix() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplInterface::GetNonlinearEffects() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplInterface::GetGravity() const {
  throw NotImplementedException();
}

const huron::Vector6d& ModelImplInterface::GetSpatialMomentum() const {
  throw NotImplementedException();
}

huron::Vector6d ModelImplInterface::GetCentroidalMomentum() const {
  throw NotImplementedException();
}

const huron::Matrix6Xd& ModelImplInterface::GetCentroidalMatrix() const {
  throw NotImplementedException();
}

void ModelImplInterface::ComputeAll(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  throw NotImplementedException();
}

void ModelImplInterface::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q) {
  throw NotImplementedException();
}
void ModelImplInterface::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  throw NotImplementedException();
}
void ModelImplInterface::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v,
  const Eigen::Ref<const Eigen::VectorXd>& a) {
  throw NotImplementedException();
}

bool ModelImplInterface::is_built() const {
  throw NotImplementedException();
}

size_t ModelImplInterface::num_positions() const {
  throw NotImplementedException();
}
size_t ModelImplInterface::num_velocities() const {
  throw NotImplementedException();
}
size_t ModelImplInterface::num_joints() const {
  throw NotImplementedException();
}
size_t ModelImplInterface::num_frames() const {
  throw NotImplementedException();
}

}  // namespace internal
}  // namespace multibody
}  // namespace huron
