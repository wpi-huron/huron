#include "huron/multibody/pinocchio_model_impl.h"
#include "huron/multibody/joint_common.h"
#include "huron/exceptions/not_implemented_exception.h"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

namespace huron {
namespace multibody {

PinocchioModelImpl::PinocchioModelImpl() = default;

void PinocchioModelImpl::BuildFromUrdf(const std::string& urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);
}

void PinocchioModelImpl::GetJointDescription(
  JointIndex joint_index,
  JointDescription& joint_desc) const {
  throw NotImplementedException("Pinocchio does not support this feature.");
}

void PinocchioModelImpl::GetJointDescriptionFromChildFrame(
  FrameIndex child_frame_index,
  JointDescription& joint_desc) const {
  size_t joint_index = model_.frames[child_frame_index].parent;
  size_t parent_frame_index = 
    model_.frames[child_frame_index].previousFrame;
  joint_desc = JointDescription(
      joint_index,
      model_.names[joint_index],
      parent_frame_index,
      child_frame_index,
      model_.nqs[joint_index],
      model_.nvs[joint_index],
      model_.lowerPositionLimit.segment(model_.idx_qs[joint_index],
                                        model_.nqs[joint_index]),
      model_.upperPositionLimit.segment(model_.idx_qs[joint_index],
                                        model_.nqs[joint_index]),
      -model_.velocityLimit.segment(model_.idx_vs[joint_index],
                                   model_.nvs[joint_index]),
      model_.velocityLimit.segment(model_.idx_vs[joint_index],
                                   model_.nvs[joint_index]),
      Eigen::VectorXd::Constant(model_.nvs[joint_index],
                                -std::numeric_limits<double>::infinity()),
      Eigen::VectorXd::Constant(model_.nvs[joint_index],
                                std::numeric_limits<double>::infinity()),
      -model_.effortLimit.segment(model_.idx_vs[joint_index],
                                 model_.nvs[joint_index]),
      model_.effortLimit.segment(model_.idx_vs[joint_index],
                                 model_.nvs[joint_index]),
      model_.friction.segment(model_.idx_vs[joint_index],
                              model_.nvs[joint_index]),
      model_.damping.segment(model_.idx_vs[joint_index],
                             model_.nvs[joint_index]));
}

JointType PinocchioModelImpl::GetJointType(size_t joint_index) const {
  if (model_.joints[joint_index].shortname() == "JointModelFreeFlyer") {
    return JointType::kFreeFlyer;
  } else if (model_.joints[joint_index].shortname() == "JointModelRX") {
    return JointType::kRevolute;
  } else if (model_.joints[joint_index].shortname() == "JointModelRY") {
    return JointType::kRevolute;
  } else if (model_.joints[joint_index].shortname() == "JointModelRZ") {
    return JointType::kRevolute;
  } else if (model_.joints[joint_index].shortname() == "JointModelPX") {
    return JointType::kPrismatic;
  } else if (model_.joints[joint_index].shortname() == "JointModelPY") {
    return JointType::kPrismatic;
  } else if (model_.joints[joint_index].shortname() == "JointModelPZ") {
    return JointType::kPrismatic;
  } else if (model_.joints[joint_index].shortname() == "JointModelSpherical") {
    return JointType::kSpherical;
  } else if (model_.joints[joint_index].shortname() == "JointModelPlanar") {
    return JointType::kPlanar;
  } else {
    return JointType::kUnknown;
  }
}

Eigen::VectorXd PinocchioModelImpl::GetAccelerations() const {
  return data_.ddq;
}

Eigen::VectorXd PinocchioModelImpl::GetTorques() const {
  return data_.tau;
}

Eigen::MatrixXd PinocchioModelImpl::GetMassMatrix() const {
  return data_.M;
}

Eigen::MatrixXd PinocchioModelImpl::GetCoriolisMatrix() const {
  return data_.C;
}

Eigen::VectorXd PinocchioModelImpl::GetNonlinearEffects() const {
  return data_.nle;
}

Eigen::VectorXd PinocchioModelImpl::GetGravity() const {
  throw data_.g;
}

huron::Vector6d PinocchioModelImpl::GetSpatialMomentum() const {
  throw NotImplementedException();
}

huron::Vector6d PinocchioModelImpl::GetCentroidalMomentum() const {
  return data_.hg;
}

huron::Matrix6Xd PinocchioModelImpl::GetCentroidalMatrix() const {
  return data_.Ag;
}

void PinocchioModelImpl::ComputeAll(
  const Eigen::Ref<const Eigen::VectorXd>& q,
  const Eigen::Ref<const Eigen::VectorXd>& v) {
  pinocchio::computeAllTerms(model_, data_, q, v);
}

}  // namespace multibody
}  // namespace huron
