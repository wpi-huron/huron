#include "huron/multibody/model.h"
#include "huron/multibody/model_impl_base.h"

namespace huron {
namespace multibody {

Model::Model() = default;

template<typename ImplType, typename ...Args>
void Model::AddModelImpl(bool set_as_default,
                         Args&&... args) {
  static_assert(
    std::is_base_of_v<internal::ModelImplBase, ImplType>,
    "type parameter of this method must derive from ModelImplInterface");
  impls_.push_back(std::make_unique<ImplType>(std::forward<Args>(args)...));
  if (set_as_default) {
    default_impl_index_ = impls_.size() - 1;
  }
}

ModelImplInterface const * Model::GetModelImpl(size_t index) const {
  return impls_[index].get();
}

void Model::BuildFromUrdf(const std::string& urdf_path) {
  if (impls_.empty()) {
    throw std::runtime_error("No model implementations.");
  }
  for (auto& impl : impls_) {
    impl->BuildFromUrdf(urdf_path);
  }
  // Initialize the joint vector
  joints_ = std::vector<std::shared_ptr<Joint>>(
      impls_[default_impl_index_]->num_joints());

  for (auto i = 0; i < impls_[default_impl_index_]->num_frames(); ++i) {
    auto jd_tmp_ptr = 
      impls_[default_impl_index_]->GetJointDescriptionFromChildFrame(i);
    JointIndex idx = jd_tmp_ptr->id();
    AddJoint(idx, std::move(jd_tmp_ptr));
  }
  is_constructed_ = true;
}

void Model::Finalize() {
  assert(is_constructed_);
  size_t last_position_index = 0, last_velocity_index = 0;
  for (auto& joint : joints_) {
    if (joint == nullptr) {
      throw std::runtime_error("Joint is not added to the model.");
    }
    joint->SetIndices(last_position_index, last_velocity_index);
    last_position_index += joint->Info()->num_positions();
    last_velocity_index += joint->Info()->num_velocities();
  }
}

template<typename ...Args>
void Model::AddJoint(JointIndex index,
                     Args&&... args) {
  assert(is_constructed_);
  if (joints_[index] != nullptr) {
    // TODO(dtbpkmte): provide index information in the error message.
    throw std::runtime_error("Joint already exists at this index.");
  }
  joints_[index] = std::make_shared<Joint>(std::forward<Args>(args)...);
}

Joint* const Model::GetJoint(JointIndex index) {
  return joints_[index].get();
}

void Model::UpdateStates() {
  for (auto& joint : joints_) {
    joint->UpdateState();
    states_.segment(joint->id_q(),
                    joint->Info()->num_positions()) = joint->GetPositions();
    states_.segment(num_positions_ + joint->id_v(),
                    joint->Info()->num_velocities()) = joint->GetVelocities();
  }
}

// Kinematics and Dynamics functions
const Eigen::Affine3d& Model::GetJointPoseInWorld(size_t joint_index) const {
  return impls_[default_impl_index_]->GetJointPoseInWorld(joint_index);
}

const FrameIndex& Model::GetFrameIndex(const std::string& frame_name) const {
  return impls_[default_impl_index_]->GetFrameIndex(frame_name);
}

const Eigen::Affine3d& Model::GetFrameTransform(FrameIndex from_frame,
                                                FrameIndex to_frame) const {
  return impls_[default_impl_index_]->GetFrameTransform(from_frame, to_frame);
}

const Eigen::Affine3d& Model::GetFrameTransformInWorld(FrameIndex frame) const {
  return impls_[default_impl_index_]->GetFrameTransformInWorld(frame);
}

const Eigen::VectorXd& Model::NeutralConfiguration() const {
  return impls_[default_impl_index_]->NeutralConfiguration();
}

const Eigen::VectorXd& Model::GetAccelerations() const {
  return impls_[default_impl_index_]->GetAccelerations();
}

const Eigen::VectorXd& Model::GetTorques() const {
  return impls_[default_impl_index_]->GetTorques();
}

const Eigen::MatrixXd& Model::GetMassMatrix() const {
  return impls_[default_impl_index_]->GetMassMatrix();
}

const Eigen::MatrixXd& Model::GetCoriolisMatrix() const {
  return impls_[default_impl_index_]->GetCoriolisMatrix();
}

const Eigen::VectorXd& Model::GetNonlinearEffects() const {
  return impls_[default_impl_index_]->GetNonlinearEffects();
}

const Eigen::VectorXd& Model::GetGravity() const {
  return impls_[default_impl_index_]->GetGravity();
}

const huron::Vector6d& Model::GetSpatialMomentum() const {
  return impls_[default_impl_index_]->GetSpatialMomentum();
}

const huron::Vector6d& Model::GetCentroidalMomentum() const {
  return impls_[default_impl_index_]->GetCentroidalMomentum();
}

const huron::Matrix6Xd& Model::GetCentroidalMatrix() const {
  return impls_[default_impl_index_]->GetCentroidalMatrix();
}


void Model::ComputeAll() {
  impls_[default_impl_index_]->ComputeAll(
    states_.segment(0, num_positions_),
    states_.segment(num_positions_, num_velocities_));
}


void Model::ForwardKinematics() {
  impls_[default_impl_index_]->ForwardKinematics(
    states_.segment(0, num_positions_),
    states_.segment(num_positions_, num_velocities_));
}


}  // namespace multibody
}  // namespace huron
