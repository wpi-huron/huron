#include "huron/multibody/com_frame.h"
#include "huron/multibody/model.h"

namespace huron {
namespace multibody {

ComFrame::ComFrame(
  FrameIndex index,
  const std::string& name,
  bool is_user_defined,
  std::weak_ptr<const Model> model,
  FrameIndex parent_frame_index)
  : Frame(index, name, FrameType::kLogical, is_user_defined, std::move(model)),
    parent_frame_index_(parent_frame_index) {}

Eigen::Affine3d ComFrame::GetTransformInWorld() const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransformInWorld(parent_frame_index_);
  return parent_transform * ParentToThisTransform();
}

Eigen::Affine3d ComFrame::GetTransformFromFrame(const Frame& other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(other.index(), parent_frame_index_);
  return parent_transform * ParentToThisTransform();
}

Eigen::Affine3d ComFrame::GetTransformFromFrame(FrameIndex other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(other, parent_frame_index_);
  return parent_transform * ParentToThisTransform();
}

Eigen::Affine3d ComFrame::GetTransformToFrame(const Frame& other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(parent_frame_index_, other.index());
  return ParentToThisTransform().inverse() * parent_transform;
}

Eigen::Affine3d ComFrame::GetTransformToFrame(FrameIndex other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(parent_frame_index_, other);
  return ParentToThisTransform().inverse() * parent_transform;
}

Eigen::Affine3d ComFrame::ParentToThisTransform() const {
  Eigen::Affine3d parent_to_this = Eigen::Affine3d::Identity();
  parent_to_this.translate(model_.lock()->GetCenterOfMassPosition());
  return parent_to_this;
}

}  // namespace multibdody
}  // namespace huron
