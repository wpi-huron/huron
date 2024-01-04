#include "huron/multibody/frame.h"
#include "huron/multibody/model.h"

namespace huron {
namespace multibody {

Frame::Frame(FrameIndex index,
             const std::string& name,
             FrameType type,
             bool is_user_defined,
             std::weak_ptr<const Model> model)
  : index_(index),
    name_(name),
    type_(type),
    is_user_defined_(is_user_defined),
    model_(std::move(model)) {}

Eigen::Affine3d Frame::GetTransformInWorld() const {
  return model_.lock()->GetFrameTransformInWorld(index_);
}

Eigen::Affine3d Frame::GetTransformFromFrame(const Frame& other) const {
  return model_.lock()->GetFrameTransform(other.index_, index_);
}

Eigen::Affine3d Frame::GetTransformFromFrame(FrameIndex other) const {
  return model_.lock()->GetFrameTransform(other, index_);
}

Eigen::Affine3d Frame::GetTransformToFrame(const Frame& other) const {
  return model_.lock()->GetFrameTransform(index_, other.index_);
}

Eigen::Affine3d Frame::GetTransformToFrame(FrameIndex other) const {
  return model_.lock()->GetFrameTransform(index_, other);
}

}  // namespace multibody
}  // namespace huron
