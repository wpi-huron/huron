#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include "huron/enable_protected_make_shared.h"

namespace huron {
namespace multibody {

class Model;

using FrameIndex = size_t;

enum class FrameType {
  kLogical,
  kFixed,
  kJoint,
  kSensor,
  kPhysical,
};

class Frame : public enable_protected_make_shared<Frame> {
 public:
  friend class Model;

  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;
  virtual ~Frame() = default;

  virtual Eigen::Affine3d GetTransformInWorld() const;
  virtual Eigen::Affine3d GetTransformFromFrame(const Frame& other) const;
  virtual Eigen::Affine3d GetTransformFromFrame(FrameIndex other) const;
  virtual Eigen::Affine3d GetTransformToFrame(const Frame& other) const;
  virtual Eigen::Affine3d GetTransformToFrame(FrameIndex other) const;

  const std::string& name() const { return name_; }
  FrameIndex index() const { return index_; }
  FrameType type() const { return type_; }
  bool is_user_defined() const { return is_user_defined_; }

 protected:
  Frame(FrameIndex index,
        const std::string& name,
        FrameType type,
        bool is_user_defined,
        std::weak_ptr<const Model> model);

  /// \brief Frame name.
  const FrameIndex index_;
  const std::string name_;
  const FrameType type_;
  bool is_user_defined_;
  const std::weak_ptr<const Model> model_;
};

}  // namespace multibody
}  // namespace huron
