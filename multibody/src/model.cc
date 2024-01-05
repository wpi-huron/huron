#include "huron/multibody/model.h"
#include "huron/multibody/model_impl_factory.h"

namespace huron {
namespace multibody {

Model::Model() = default;

void Model::AddModelImpl(ModelImplType type,
                         bool set_as_default) {
  impls_.push_back(internal::ModelImplFactory::Create(type));
  if (set_as_default) {
    default_impl_index_ = impls_.size() - 1;
  }
}

internal::ModelImplInterface const * Model::GetModelImpl(size_t index) const {
  return impls_[index].get();
}

void Model::BuildFromUrdf(const std::string& urdf_path,
                          JointType root_joint_type) {
  assert(!is_finalized_);
  if (impls_.empty()) {
    throw std::runtime_error("No model implementations.");
  }
  for (auto& impl : impls_) {
    impl->BuildFromUrdf(urdf_path, root_joint_type);
  }
  num_positions_ = impls_[default_impl_index_]->num_positions();
  num_velocities_ = impls_[default_impl_index_]->num_velocities();
  // Initialize the joint vector
  joints_ = std::vector<std::shared_ptr<Joint>>(
      impls_[default_impl_index_]->num_joints());
  // Initialize the frame vector
  frames_ = std::vector<std::shared_ptr<Frame>>(
      impls_[default_impl_index_]->num_frames());

  auto joint_names = impls_[default_impl_index_]->GetJointNames();
  for (auto i = 0; i < joints_.size(); ++i) {
    auto jd_tmp_ptr = 
      impls_[default_impl_index_]->GetJointDescription(joint_names[i]);
    // Add joint
    AddJoint(i, std::move(jd_tmp_ptr));
  }
  for (auto i = 0; i < frames_.size(); ++i) {
    // Add frame
    DoAddFrameFromModelDescription(
      i,
      impls_[default_impl_index_]->GetFrameName(i),
      impls_[default_impl_index_]->GetFrameType(i));
  }
  is_constructed_ = true;
}

void Model::Finalize(const Eigen::VectorXd& initial_state) {
  assert(is_constructed_);
  // Check if all joints are added to the model, and set the joint indices.
  size_t last_position_index = 0, last_velocity_index = 0;
  for (auto& joint : joints_) {
    assert(joint != nullptr);
    assert(joint->IsFullyConfigured());
    joint->SetIndices(last_position_index, last_velocity_index);
    last_position_index += joint->Info()->num_positions();
    last_velocity_index += joint->Info()->num_velocities();
  }
  // // Resize the state vector
  // states_.resize(num_positions_ + num_velocities_);
  // Set the initial state
  assert((initial_state.size() == num_positions_ + num_velocities_) &&
         "The initial state dimension does not match the model.");
  states_ = initial_state;
  is_finalized_ = true;
}

void Model::Finalize() {
  assert(is_constructed_);
  Model::Finalize(Eigen::VectorXd::Zero(num_positions_ + num_velocities_));
}

Joint* const Model::GetJoint(JointIndex index) {
  assert(is_finalized_);
  return joints_[index].get();
}

Joint* const Model::GetJoint(const std::string& name) {
  assert(is_finalized_);
  return joints_[GetJointIndex(name)].get();
}

void Model::SetJointStateProvider(
  JointIndex index,
  std::shared_ptr<StateProvider> state_provider) {
  assert(is_constructed_);
  joints_[index]->SetStateProvider(std::move(state_provider));
}

JointIndex Model::GetJointIndex(const std::string& joint_name) const {
  return impls_[default_impl_index_]->GetJointIndex(joint_name);
}

void Model::DoAddFrameFromModelDescription(FrameIndex idx,
                                           const std::string& name,
                                           FrameType type) {
  frames_[idx] = Frame::make_shared(
    idx,
    name,
    type,
    false,
    weak_from_this());
  frame_name_to_index_[name] = idx;
}

void Model::UpdateJointStates() {
  assert(is_finalized_);
  for (auto& joint : joints_) {
    if (joint->Info()->type() == JointType::kUnknown) {
      continue;
    }
    joint->UpdateState();
    states_.segment(joint->id_q(),
                    joint->Info()->num_positions()) = joint->GetPositions();
    states_.segment(num_positions_ + joint->id_v(),
                    joint->Info()->num_velocities()) = joint->GetVelocities();
  }
}

std::weak_ptr<const Frame> Model::GetFrame(FrameIndex index) const {
  return frames_[index];
}

std::weak_ptr<const Frame> Model::GetFrame(const std::string& name) const {
  return frames_[frame_name_to_index_.at(name)];
}

// Kinematics and Dynamics functions
Eigen::Affine3d Model::GetJointTransformInWorld(size_t joint_index) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetJointTransformInWorld(joint_index);
}

FrameIndex Model::GetFrameIndex(const std::string& frame_name) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetFrameIndex(frame_name);
}

Eigen::Affine3d Model::GetFrameTransform(FrameIndex from_frame,
                                         FrameIndex to_frame) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetFrameTransform(from_frame, to_frame);
}

Eigen::Affine3d Model::GetFrameTransformInWorld(FrameIndex frame) const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetFrameTransformInWorld(frame);
}

Eigen::VectorXd Model::NeutralConfiguration() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->NeutralConfiguration();
}

const Eigen::VectorXd& Model::GetAccelerations() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetAccelerations();
}

const Eigen::VectorXd& Model::GetTorques() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetTorques();
}

const Eigen::MatrixXd& Model::GetMassMatrix() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetMassMatrix();
}

const Eigen::MatrixXd& Model::GetCoriolisMatrix() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCoriolisMatrix();
}

const Eigen::VectorXd& Model::GetNonlinearEffects() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetNonlinearEffects();
}

const Eigen::VectorXd& Model::GetGravity() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetGravity();
}

const huron::Vector6d& Model::GetSpatialMomentum() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetSpatialMomentum();
}

huron::Vector6d Model::GetCentroidalMomentum() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCentroidalMomentum();
}

const huron::Matrix6Xd& Model::GetCentroidalMatrix() const {
  assert(is_finalized_);
  return impls_[default_impl_index_]->GetCentroidalMatrix();
}


void Model::ComputeAll() {
  assert(is_finalized_);
  impls_[default_impl_index_]->ComputeAll(
    states_.segment(0, num_positions_),
    states_.segment(num_positions_, num_velocities_));
}


void Model::ForwardKinematics() {
  assert(is_finalized_);
  impls_[default_impl_index_]->ForwardKinematics(
    states_.segment(0, num_positions_),
    states_.segment(num_positions_, num_velocities_));
}

}  // namespace multibody
}  // namespace huron
