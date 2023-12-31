#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "huron/control/push_recovery.h"

// Some basic methods testing
TEST(PushRecoveryTest, ZeroConfiguration) {
  PushRecoveryControl pr;
  // Expect equality.
  EXPECT_EQ(pr.GetTorque({1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
            Eigen::Vector3d::Zero());
}
