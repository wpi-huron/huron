#include "huron/math/rotation.h"

namespace huron {
namespace math {

Eigen::Vector3d ZyxToRpy(
  const Eigen::Ref<const Eigen::Vector3d>& zyx) {
  return zyx.reverse();
}

Eigen::Vector3d RpyToZyx(
  const Eigen::Ref<const Eigen::Vector3d>& rpy) {
  return rpy.reverse();
}

Eigen::Matrix3d ZyxToRotationMatrix(
  const Eigen::Ref<const Eigen::Vector3d>& zyx) {
  using std::cos, std::sin;
  double a = zyx(0), b = zyx(1), g = zyx(2);
  double ca = cos(a), cb = cos(b), cg = cos(g);
  double sa = sin(a), sb = sin(b), sg = sin(g);
  return (Eigen::Matrix3d() <<  ca*cb,
                                ca*sb*sg - sa*cg,
                                ca*sb*cg + sa*sg,

                                sa*cb,
                                sa*sb*sg + ca*cg,
                                sa*sb*cg - ca*sg,

                                -sb,
                                cb*sg,
                                cb*cg).finished();
}

Eigen::Vector3d RotationMatrixToZyx(
  const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix) {
  using std::atan2;
  Eigen::Vector3d zyx;
  if (rotation_matrix(0, 0) == 0.0 && rotation_matrix(1, 0) == 0.0) {
    zyx << 0, M_PI_2, std::atan2(rotation_matrix(0, 1), rotation_matrix(1, 1));
  } else {
    zyx <<  atan2(rotation_matrix(1, 0), rotation_matrix(0, 0)),
            atan2(-rotation_matrix(2, 0),
                  rotation_matrix.block<2, 1>(0, 0).norm()),
            atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
  }
  return zyx;
}

Eigen::Matrix3d RpyToRotationMatrix(
  const Eigen::Ref<const Eigen::Vector3d>& rpy) {
  return ZyxToRotationMatrix(rpy.reverse());
}

Eigen::Vector3d RotationMatrixToRpy(
  const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix) {
  return RotationMatrixToZyx(rotation_matrix).reverse();
}


}  // namespace math
}  // namespace huron
