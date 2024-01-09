#pragma once

#include <eigen3/Eigen/Core>

namespace huron {

typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 6, Eigen::Dynamic > Matrix6Xd;

}  // namespace huron
