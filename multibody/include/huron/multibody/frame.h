#pragma once

#include <eigen3/Eigen/Dense>

namespace huron {
namespace multibody {

using FrameIndex = size_t;

class Frame {
 public:
  Frame(const std::string& name,
        const Eigen::Affine3d& pose);

 private:
  /// \brief Frame name.
  const std::string name_;
  /// \brief Pose of the frame w.r.t. the parent frame.
  Eigen::Affine3d pose_;
};

}  // namespace multibody
}  // namespace huron
