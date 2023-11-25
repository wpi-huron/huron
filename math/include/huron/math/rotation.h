#pragma once

#include <eigen3/Eigen/Core>
#include <cmath>

namespace huron {
namespace math {

Eigen::Vector3d ZyxToRpy(
  const Eigen::Ref<const Eigen::Vector3d>& zyx);

Eigen::Vector3d RpyToZyx(
  const Eigen::Ref<const Eigen::Vector3d>& zyx);

Eigen::Matrix3d ZyxToRotationMatrix(
  const Eigen::Ref<const Eigen::Vector3d>& zyx);

Eigen::Vector3d RotationMatrixToZyx(
  const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix);

Eigen::Matrix3d RpyToRotationMatrix(
  const Eigen::Ref<const Eigen::Vector3d>& rpy);

Eigen::Vector3d RotationMatrixToRpy(
  const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix);


}  // namespace math
}  // namespace huron
