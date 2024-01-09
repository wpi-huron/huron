#include "huron/multibody/logical_frame.h"
#include "huron/multibody/model.h"

namespace huron {
namespace multibody {

LogicalFrame::LogicalFrame(
  FrameIndex index,
  const std::string& name,
  bool is_user_defined,
  std::weak_ptr<const Model> model,
  FrameIndex parent_frame_index,
  std::function<Eigen::Affine3d(const Eigen::Affine3d&)> transform_function)
  : Frame(index, name, FrameType::kLogical, is_user_defined, std::move(model)),
    parent_frame_index_(parent_frame_index),
    transform_function_(transform_function) {}

Eigen::Affine3d LogicalFrame::GetTransformInWorld() const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransformInWorld(parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

Eigen::Affine3d LogicalFrame::GetTransformFromFrame(const Frame& other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(other.index(), parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

Eigen::Affine3d LogicalFrame::GetTransformFromFrame(FrameIndex other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(other, parent_frame_index_);
  return parent_transform * transform_function_(parent_transform);
}

Eigen::Affine3d LogicalFrame::GetTransformToFrame(const Frame& other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(parent_frame_index_, other.index());
  return transform_function_(parent_transform).inverse() * parent_transform;
}

Eigen::Affine3d LogicalFrame::GetTransformToFrame(FrameIndex other) const {
  Eigen::Affine3d parent_transform =
    model_.lock()->GetFrameTransform(parent_frame_index_, other);
  return transform_function_(parent_transform).inverse() * parent_transform;
}

}  // namespace multibdody
}  // namespace huron
