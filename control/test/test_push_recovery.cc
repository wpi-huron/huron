#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "huron/control/push_recovery.h"

// Some basic methods testing
TEST(PushRecoveryTest, ZeroConfiguration) {
  PushRecoveryControl pr;
  Eigen::Vector2d cop(0, 0);
  Eigen::VectorXd position = Eigen::VectorXd::Zero(7 + 12);
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(6 + 12);
  // Expect equality.
  EXPECT_EQ(pr.GetTorque(cop, position, velocity),
            Eigen::Vector3d::Zero());
}
