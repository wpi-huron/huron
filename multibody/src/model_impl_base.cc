#include "huron/multibody/model_impl_base.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

ModelImplBase::ModelImplBase() = default;

void ModelImplBase::BuildFromUrdf(const std::string& urdf_path) {
  throw NotImplementedException();
}

std::vector<std::string> ModelImplBase::GetJointNames() const {
  throw NotImplementedException();
}

std::weak_ptr<Joint> ModelImplBase::GetJoint(const std::string& name) const {
  throw NotImplementedException();
}

std::weak_ptr<Joint> ModelImplBase::GetJoint(size_t joint_index) const {
  throw NotImplementedException();
}

std::unique_ptr<JointDescription> ModelImplBase::GetJointDescription(
  JointIndex joint_index) const {
  throw NotImplementedException();
}

std::unique_ptr<JointDescription>
ModelImplBase::GetJointDescriptionFromChildFrame(
  FrameIndex child_frame_index) const {
  throw NotImplementedException();
}


const Eigen::Affine3d& ModelImplBase::GetJointTransformInWorld(
  size_t joint_index) const {
  throw NotImplementedException();
}


const FrameIndex& ModelImplBase::GetFrameIndex(const std::string& frame_name) const {
  throw NotImplementedException();
}

const Eigen::Affine3d& ModelImplBase::GetFrameTransform(FrameIndex from_frame,
                                         FrameIndex to_frame) const {
  throw NotImplementedException();
}

const Eigen::Affine3d&
ModelImplBase::GetFrameTransformInWorld(FrameIndex frame) const {
  throw NotImplementedException();
}


const Eigen::VectorXd& ModelImplBase::GetAccelerations() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplBase::GetTorques() const {
  throw NotImplementedException();
}

const Eigen::MatrixXd& ModelImplBase::GetMassMatrix() const {
  throw NotImplementedException();
}

const Eigen::MatrixXd& ModelImplBase::GetCoriolisMatrix() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplBase::GetNonlinearEffects() const {
  throw NotImplementedException();
}

const Eigen::VectorXd& ModelImplBase::GetGravity() const {
  throw NotImplementedException();
}

const huron::Vector6d& ModelImplBase::GetSpatialMomentum() const {
  throw NotImplementedException();
}

const huron::Vector6d& ModelImplBase::GetCentroidalMomentum() const {
  throw NotImplementedException();
}

const huron::Matrix6d& ModelImplBase::GetCentroidalMatrix() const {
  throw NotImplementedException();
}

void ModelImplBase::ComputeAll(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  throw NotImplementedException();
}

void ModelImplBase::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q) {
  throw NotImplementedException();
}

void ModelImplBase::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  throw NotImplementedException();
}

void ModelImplBase::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v,
  const Eigen::Ref<const Eigen::VectorXd>& a) {
  throw NotImplementedException();
}


}  // namespace internal
}  // namespace multibody
}  // namespace huron
