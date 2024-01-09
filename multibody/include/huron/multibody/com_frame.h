#pragma once

#include "huron/multibody/frame.h"

namespace huron {
namespace multibody {

/**
 * @brief Robot center of mass frame.
 */
class ComFrame : public Frame {
 public:
  ComFrame(FrameIndex index,
           const std::string& name,
           bool is_user_defined,
           std::weak_ptr<const Model> model,
           FrameIndex parent_frame_index);

  ComFrame(const ComFrame&) = delete;
  ComFrame& operator=(const ComFrame&) = delete;
  ~ComFrame() override = default;

  Eigen::Affine3d GetTransformInWorld() const override;
  Eigen::Affine3d GetTransformFromFrame(const Frame& other) const override;
  Eigen::Affine3d GetTransformFromFrame(FrameIndex other) const override;
  Eigen::Affine3d GetTransformToFrame(const Frame& other) const override;
  Eigen::Affine3d GetTransformToFrame(FrameIndex other) const override;

 private:
  FrameIndex parent_frame_index_;

  Eigen::Affine3d ParentToThisTransform() const;
};

}  // namespace multibody
}  // namespace huron
