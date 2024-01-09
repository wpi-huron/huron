#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "huron/math/rotation.h"

using namespace huron::math;  //NOLINT

TEST(MathRotationTest, ZeroRotation) {
  double tolerance = 0.0001;
  Eigen::Vector3d angles;
  angles.setZero();
  Eigen::Matrix3d identity;
  identity.setIdentity();

  EXPECT_EQ(RpyToRotationMatrix(angles),
            Eigen::Matrix3d::Identity());
  EXPECT_EQ(ZyxToRotationMatrix(angles),
            Eigen::Matrix3d::Identity());
  EXPECT_EQ(RotationMatrixToRpy(identity),
            angles);
  EXPECT_EQ(RotationMatrixToZyx(identity),
            angles);
  EXPECT_LE((RotationMatrixToZyx(ZyxToRotationMatrix(angles)) - angles).norm(),
            tolerance);
}

TEST(MathRotationTest, HalfPiRotation) {
  double tolerance = 0.0001;
  Eigen::Vector3d angles;
  angles << M_PI_2, M_PI_2, M_PI_2;
  Eigen::Matrix3d expected;
  expected << 0.0, 0.0, 1.0,
              0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0;

  EXPECT_LE((huron::math::ZyxToRotationMatrix(angles) - expected).norm(),
            tolerance);
}

TEST(MathRotationTest, ArbitraryRotation) {
  double tolerance = 0.0001;
  Eigen::Vector3d angles;
  angles << 0.957506835434298,
            0.546881519204984,
            0.278498218867048;
  Eigen::Matrix3d expected;
  expected << 0.491615279747179, -0.703967407781224, 0.512585900606803,
              0.698489270695994, 0.670291899863360, 0.250642190582803,
              -0.520026110346088, 0.234816221244084, 0.821239421118450;
  EXPECT_LE((huron::math::ZyxToRotationMatrix(angles) - expected).norm(),
            tolerance);
}
