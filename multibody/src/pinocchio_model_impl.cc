#include "huron/multibody/pinocchio_model_impl.h"
#include "huron/multibody/joint_common.h"
#include "huron/multibody/model_impl_types.h"
#include "huron/exceptions/not_implemented_exception.h"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

namespace huron {
namespace multibody {
namespace internal {

namespace helpers {

Eigen::Affine3d Se3ToAffine3d(const pinocchio::SE3& se3) {
  Eigen::Affine3d affine;
  affine.linear() = se3.rotation();
  affine.translation() = se3.translation();
  return affine;
}

}  // namespace helpers

using namespace helpers;

struct PinocchioModelImpl::Impl {
  mutable pinocchio::Model model_;
  mutable pinocchio::Data data_;
};

PinocchioModelImpl::PinocchioModelImpl()
  : impl_(std::make_unique<Impl>()) {}

PinocchioModelImpl::~PinocchioModelImpl() = default;

void PinocchioModelImpl::BuildFromUrdf(const std::string& urdf_path,
                                       JointType root_joint_type) {
  pinocchio::Model::JointModel joint_model;
  if (root_joint_type == JointType::kFixed) {
    pinocchio::urdf::buildModel(urdf_path, impl_->model_);
  } else {
    switch (root_joint_type) {
      case JointType::kFreeFlyer:
        joint_model = pinocchio::JointModelFreeFlyer();
        break;
      case JointType::kPlanar:
        joint_model = pinocchio::JointModelPlanar();
        break;
      default:
        throw std::runtime_error("Unsupported root joint type.");
        break;
    }
    pinocchio::urdf::buildModel(urdf_path, joint_model, impl_->model_);
  }
  impl_->data_ = pinocchio::Data(impl_->model_);
  num_positions_ = impl_->model_.nq;
  num_velocities_ = impl_->model_.nv;
  num_joints_ = impl_->model_.njoints;
  num_frames_ = impl_->model_.nframes;
}

const std::vector<std::string>& PinocchioModelImpl::GetJointNames() const {
  return impl_->model_.names;
}

std::weak_ptr<Joint> PinocchioModelImpl::GetJoint(const std::string& name) const {
  throw NotImplementedException();
}
std::weak_ptr<Joint> PinocchioModelImpl::GetJoint(size_t joint_index) const {
  throw NotImplementedException();
}

std::unique_ptr<JointDescription> PinocchioModelImpl::GetJointDescription(
  JointIndex joint_index) const {
  return GetJointDescription(impl_->model_.names[joint_index]);
}

std::unique_ptr<JointDescription>
PinocchioModelImpl::GetJointDescription(
  const std::string& joint_name) const {
  // auto frame_id = impl_->model_.getFrameId(joint_name, pinocchio::JOINT);
  auto frame_id = impl_->model_.getFrameId(joint_name);
  assert(frame_id < impl_->model_.nframes);
  auto frame = impl_->model_.frames[frame_id];
  size_t joint_index = frame.parent;
  size_t parent_frame_index =  frame.previousFrame;
  JointType joint_type = (frame_id == 0) ? JointType::kUnknown
                                         : GetJointType(joint_index);
  return std::make_unique<JointDescription>(
    frame.parent,
    joint_name,
    parent_frame_index,
    frame_id + 1,  // child frame seems to be the immediate next frame
    impl_->model_.nqs[joint_index],
    impl_->model_.nvs[joint_index],
    joint_type,
    impl_->model_.lowerPositionLimit.segment(impl_->model_.idx_qs[joint_index],
                                      impl_->model_.nqs[joint_index]),
    impl_->model_.upperPositionLimit.segment(impl_->model_.idx_qs[joint_index],
                                      impl_->model_.nqs[joint_index]),
    -impl_->model_.velocityLimit.segment(impl_->model_.idx_vs[joint_index],
                                 impl_->model_.nvs[joint_index]),
    impl_->model_.velocityLimit.segment(impl_->model_.idx_vs[joint_index],
                                 impl_->model_.nvs[joint_index]),
    Eigen::VectorXd::Constant(impl_->model_.nvs[joint_index],
                              -std::numeric_limits<double>::infinity()),
    Eigen::VectorXd::Constant(impl_->model_.nvs[joint_index],
                              std::numeric_limits<double>::infinity()),
    -impl_->model_.effortLimit.segment(impl_->model_.idx_vs[joint_index],
                               impl_->model_.nvs[joint_index]),
    impl_->model_.effortLimit.segment(impl_->model_.idx_vs[joint_index],
                               impl_->model_.nvs[joint_index]),
    impl_->model_.friction.segment(impl_->model_.idx_vs[joint_index],
                            impl_->model_.nvs[joint_index]),
    impl_->model_.damping.segment(impl_->model_.idx_vs[joint_index],
                           impl_->model_.nvs[joint_index]));
}

Eigen::Affine3d PinocchioModelImpl::GetJointTransformInWorld(size_t joint_index) const {
  return helpers::Se3ToAffine3d(impl_->data_.oMi[joint_index]);
}

JointIndex PinocchioModelImpl::GetJointIndex(const std::string& joint_name) const {
  return impl_->model_.getJointId(joint_name);
}

FrameIndex PinocchioModelImpl::GetFrameIndex(
  const std::string& frame_name) const {
  return impl_->model_.getFrameId(frame_name);
}

const std::string& PinocchioModelImpl::GetFrameName(FrameIndex frame_index) const {
  return impl_->model_.frames[frame_index].name;
}

FrameType PinocchioModelImpl::GetFrameType(FrameIndex frame_index) const {
  if (impl_->model_.frames[frame_index].type == pinocchio::BODY) {
    return FrameType::kPhysical;
  } else if (impl_->model_.frames[frame_index].type == pinocchio::JOINT) {
    return FrameType::kJoint;
  } else if (impl_->model_.frames[frame_index].type == pinocchio::SENSOR) {
    return FrameType::kSensor;
  } else if (impl_->model_.frames[frame_index].type == pinocchio::FIXED_JOINT){
    return FrameType::kFixed;
  } else {
    throw std::runtime_error("Unknown frame type.");
  }
}

Eigen::Affine3d
PinocchioModelImpl::GetFrameTransform(FrameIndex from_frame,
                                      FrameIndex to_frame) const {
  return GetFrameTransformInWorld(from_frame).inverse() *
         GetFrameTransformInWorld(to_frame);
}

Eigen::Affine3d PinocchioModelImpl::GetFrameTransformInWorld(FrameIndex frame) const {
  pinocchio::updateFramePlacement(impl_->model_, impl_->data_, (size_t) frame);
  return Se3ToAffine3d(impl_->data_.oMf[frame]);
}

Eigen::Vector3d PinocchioModelImpl::EvalCenterOfMassPosition() {
  return pinocchio::centerOfMass(impl_->model_, impl_->data_);
}

Eigen::Vector3d PinocchioModelImpl::GetCenterOfMassPosition() const {
  return impl_->data_.com[0];
}

Eigen::VectorXd PinocchioModelImpl::NeutralConfiguration() const {
  return pinocchio::neutral(impl_->model_);
}

const Eigen::VectorXd& PinocchioModelImpl::GetAccelerations() const {
  return impl_->data_.ddq;
}
const Eigen::VectorXd& PinocchioModelImpl::GetTorques() const {
  return impl_->data_.tau;
}
const Eigen::MatrixXd& PinocchioModelImpl::GetMassMatrix() const {
  return impl_->data_.M;
}
const Eigen::MatrixXd& PinocchioModelImpl::GetCoriolisMatrix() const {
  return impl_->data_.C;
}
const Eigen::VectorXd& PinocchioModelImpl::GetNonlinearEffects() const {
  return impl_->data_.nle;
}
const Eigen::VectorXd& PinocchioModelImpl::GetGravity() const {
  return impl_->data_.g;
}
const huron::Vector6d& PinocchioModelImpl::GetSpatialMomentum() const {
  throw NotImplementedException();
}
huron::Vector6d PinocchioModelImpl::GetCentroidalMomentum() const {
  return impl_->data_.hg;
}
const huron::Matrix6Xd& PinocchioModelImpl::GetCentroidalMatrix() const {
  return impl_->data_.Ag;
}

void PinocchioModelImpl::ComputeAll(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  pinocchio::computeAllTerms(impl_->model_, impl_->data_, q, v);
}

void PinocchioModelImpl::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q) {
  pinocchio::forwardKinematics(impl_->model_, impl_->data_, q);
};
void PinocchioModelImpl::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  pinocchio::forwardKinematics(impl_->model_, impl_->data_, q, v);
}
void PinocchioModelImpl::ForwardKinematics(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v,
  const Eigen::Ref<const Eigen::VectorXd>& a) {
  pinocchio::forwardKinematics(impl_->model_, impl_->data_, q, v, a);
}

JointType PinocchioModelImpl::GetJointType(size_t joint_index) const {
  if (impl_->model_.joints[joint_index].shortname() == "JointModelFreeFlyer") {
    return JointType::kFreeFlyer;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelRX") {
    return JointType::kRevolute;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelRY") {
    return JointType::kRevolute;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelRZ") {
    return JointType::kRevolute;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPX") {
    return JointType::kPrismatic;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPY") {
    return JointType::kPrismatic;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPZ") {
    return JointType::kPrismatic;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelSpherical") {
    return JointType::kSpherical;
  } else if (impl_->model_.joints[joint_index].shortname() == "JointModelPlanar") {
    return JointType::kPlanar;
  } else {
    return JointType::kUnknown;
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace huron
