#pragma once

#include "huron/multibody/frame.h"

namespace huron {
namespace multibody {

/**
 * @brief A frame that is defined relative to another frame by an affine
 * transformation. This transformation is user-defined using a function that
 * takes the parent frame's transform in world coordinates as an argument and
 * returns the transform from the parent frame to this frame.
 *
 * @note This class can only be instantiated by the Model class using
 * AddLogicalFrame().
 *
 * @param index The index of this frame.
 * @param name The name of this frame.
 * @param model The model that this frame is a part of.
 * @param parent_frame_index The index of the frame that this frame is defined
 * relative to.
 * @param transform_function The function that defines the transformation from
 * the parent frame to this frame.
 */
class LogicalFrame : public Frame, enable_protected_make_shared<LogicalFrame> {
 public:
  friend class Model;

  LogicalFrame(const LogicalFrame&) = delete;
  LogicalFrame& operator=(const LogicalFrame&) = delete;
  ~LogicalFrame() override = default;

  Eigen::Affine3d GetTransformInWorld() const override;
  Eigen::Affine3d GetTransformFromFrame(const Frame& other) const override;
  Eigen::Affine3d GetTransformFromFrame(FrameIndex other) const override;
  Eigen::Affine3d GetTransformToFrame(const Frame& other) const override;
  Eigen::Affine3d GetTransformToFrame(FrameIndex other) const override;

 protected:
  LogicalFrame(FrameIndex index,
               const std::string& name,
               bool is_user_defined,
               std::weak_ptr<const Model> model,
               FrameIndex parent_frame_index,
               std::function<Eigen::Affine3d(const Eigen::Affine3d&)>
                 transform_function);

 private:
  FrameIndex parent_frame_index_;
  const std::function<Eigen::Affine3d(const Eigen::Affine3d&)>
    transform_function_;
};

}  // namespace multibody
}  // namespace huron
