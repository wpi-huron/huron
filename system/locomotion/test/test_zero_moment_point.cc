#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "huron/sensors/force_torque.h"
#include "huron/locomotion/zero_moment_point.h"

using namespace huron;  //NOLINT

class FakeForceTorqueSensor : public ForceTorqueSensor {
 public:
  FakeForceTorqueSensor(std::string name,
                        bool reverse_wrench_direction,
                        const Vector6d& fake_wrench)  
    : ForceTorqueSensor(name, reverse_wrench_direction),
      fake_wrench_(fake_wrench) {}

  void SetFakeWrench(const Vector6d& fake_wrench) {
    fake_wrench_ = fake_wrench;
  }

  Vector6d GetWrenchRaw() override {
    return fake_wrench_;
  }

  // GenericComponent interface
  void Initialize() override {}

  void SetUp() override {}

  void Terminate() override {}


 private:
  Vector6d fake_wrench_;
};

/**
 * Note: This test is using the FT sensor frame in Huron URDF. The order of
 * forces in the wrench can be found by investigating the correct ft_sensor
 * topic.
 */
TEST(ZeroMomentPointTest, ZmpFtSensorTest) {
  // Initialize objects
  double tolerance = 0.00001;
  double normal_force_threshold = 0.01;  // N
  Eigen::Vector3d sensor_position, sensor_frame_zyx;
  sensor_position << 0.0, 0.0, 0.0983224252792114;  // At a height from ground
  sensor_frame_zyx << 0.0, M_PI_2, 0.0;  // Rotate according to Huron URDF

  double expected_fz = 0.0;
  Eigen::Vector2d expected_zmp;
  expected_zmp << 0.0, 0.0;

  auto ft_sensor = std::make_shared<FakeForceTorqueSensor>(
      "sensor",
      false,  // reverse wrench direction
      (Vector6d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());

  std::shared_ptr<ZeroMomentPoint> zmp =
    std::make_shared<ZeroMomentPointFTSensor>("z0",
                                              normal_force_threshold,
                                              sensor_position,
                                              sensor_frame_zyx,
                                              ft_sensor);
  // Start testing
  // Zero force
  double fz = 1.0;
  Eigen::Vector2d result;
  ft_sensor->SetFakeWrench(Vector6d::Zero());
  zmp->Compute(result, fz);
  EXPECT_LE((result - expected_zmp).norm(), tolerance);
  EXPECT_DOUBLE_EQ(fz, expected_fz);
  EXPECT_LE((zmp->Compute() - expected_zmp).norm(), tolerance);

  // Small normal force
  expected_fz = 0.0005;
  ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  zmp->Compute(result, fz);
  EXPECT_LE((result - expected_zmp).norm(), tolerance);
  EXPECT_DOUBLE_EQ(fz, expected_fz);
  EXPECT_LE((zmp->Compute() - expected_zmp).norm(), tolerance);

  // Big normal force
  expected_fz = 1.0;
  ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  zmp->Compute(result, fz);
  EXPECT_LE((result - expected_zmp).norm(), tolerance);
  EXPECT_DOUBLE_EQ(fz, expected_fz);
  EXPECT_LE((zmp->Compute() - expected_zmp).norm(), tolerance);

  // Big normal force, with x/y components
  expected_fz = 1.0;
  expected_zmp << -2.049161212639606, 0.980335514944158;
  ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz, 0.2, 0.5, 0.0, 2.0, 1.0).finished());
  zmp->Compute(result, fz);
  EXPECT_LE((result - expected_zmp).norm(), tolerance);
  EXPECT_DOUBLE_EQ(fz, expected_fz);
  EXPECT_LE((zmp->Compute() - expected_zmp).norm(), tolerance);
}
