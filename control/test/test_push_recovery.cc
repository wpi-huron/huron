#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "huron/control/push_recovery.h"

// Some basic methods testing
TEST(PushRecoveryTest, ZeroConfiguration) {
  PushRecoveryControl pr;
  // Expect equality.
  EXPECT_EQ(pr.GetTorque(0.0,                       // x_cop
                         std::vector<double>(12),   // position
                         std::vector<double>(12)),  // velocity
            Eigen::Vector3d::Zero());
}
