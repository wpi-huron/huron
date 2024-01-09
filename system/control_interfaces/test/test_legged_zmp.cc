#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "huron/control_interfaces/legged_robot.h"
#include "huron/sensors/force_torque_sensor.h"
#include "huron/control_interfaces/constant_state_provider.h"
#include "huron/locomotion/zero_moment_point_ft_sensor.h"

using namespace huron;  //NOLINT

class TestLeggedRobot : public LeggedRobot {
 public:
  TestLeggedRobot() : LeggedRobot()  {}
  ~TestLeggedRobot() override = default;

  bool Move(const std::vector<double>& values) override {return true;}
  bool Move(const Eigen::VectorXd& values) override {return true;}
  bool Stop() override {return true;}

  void Initialize() override {}
  void SetUp() override {}
  void Terminate() override {}
};

class FakeForceTorqueSensor : public ForceTorqueSensor {
 public:
  FakeForceTorqueSensor(bool reverse_wrench_direction,
                        std::weak_ptr<const multibody::Frame> frame,
                        const Vector6d& fake_wrench)  
    : ForceTorqueSensor(reverse_wrench_direction, std::move(frame)),
      fake_wrench_(fake_wrench) {}

  void SetFakeWrench(const Vector6d& fake_wrench) {
    fake_wrench_ = fake_wrench;
  }

  // GenericComponent interface
  void Initialize() override {}
  void SetUp() override {}
  void Terminate() override {}

 protected:
  Vector6d DoGetWrenchRaw() override {
    return fake_wrench_;
  }

 private:
  Vector6d fake_wrench_;
};

class TestLeggedZmp : public testing::Test {
 protected:
  void SetUp() override {
#ifdef HURON_USE_PINOCCHIO
    multibody::ModelImplType model_impl_type =
      multibody::ModelImplType::kPinocchio;
#endif
    Eigen::Vector<double, 13> initial_state;
    initial_state << 0.0, 0.0, 1.123, 0.0, 0.0, 0.0, 1.0,  // positions
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // velocities
    robot.GetModel()->AddModelImpl(
      model_impl_type, true);
    robot.GetModel()->BuildFromUrdf("huron.urdf",
                                    multibody::JointType::kFreeFlyer);
    auto floating_joint_sp =
      std::make_shared<ConstantStateProvider>(initial_state);
    robot.RegisterStateProvider(floating_joint_sp, true);
    robot.GetModel()->SetJointStateProvider(1, floating_joint_sp);
    // Fake joint states
    for (size_t joint_index = 2; joint_index <= 13; ++joint_index) {
      auto sp =
        std::make_shared<ConstantStateProvider>(Eigen::Vector2d::Zero());
      robot.RegisterStateProvider(sp, true);
      robot.GetModel()->SetJointStateProvider(
          joint_index,
          sp);
    }
    // Fake FT sensors
    l_ft_sensor = std::make_shared<FakeForceTorqueSensor>(
        false,  // reverse wrench direction
        robot.GetModel()->GetFrame("l_ankle_roll_joint"),
        (Vector6d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    robot.RegisterStateProvider(l_ft_sensor);
    r_ft_sensor = std::make_shared<FakeForceTorqueSensor>(
        false,  // reverse wrench direction
        robot.GetModel()->GetFrame("r_ankle_roll_joint"),
        (Vector6d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    robot.RegisterStateProvider(r_ft_sensor);
    robot.GetModel()->Finalize();
    // Total ZMP
    ft_sensor_list.push_back(l_ft_sensor);
    ft_sensor_list.push_back(r_ft_sensor);
    std::shared_ptr<ZeroMomentPoint> zmp =
      std::make_shared<ZeroMomentPointFTSensor>(
        robot.GetModel()->GetFrame("universe"),
        0.005,
        ft_sensor_list);
    robot.InitializeZmp(zmp);
  }

  double normal_force_threshold = 0.01;
  double tolerance = 0.0005;

  TestLeggedRobot robot;
  std::vector<std::shared_ptr<ForceTorqueSensor>> ft_sensor_list;
  std::shared_ptr<FakeForceTorqueSensor> l_ft_sensor, r_ft_sensor;
};

TEST_F(TestLeggedZmp, TestZeroForce) {
  // Parameters
  double expected_fz, computed_fz;
  Eigen::Vector2d expected_zmp;

  // Start testing
  // Zero force
  l_ft_sensor->SetFakeWrench(Vector6d::Zero());
  r_ft_sensor->SetFakeWrench(Vector6d::Zero());

  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();
  auto com = robot.GetModel()->EvalCenterOfMassPosition();

  auto total_zmp_val = robot.EvalZeroMomentPoint();
  std::cout << (total_zmp_val - com.segment(0, 2)).transpose() << std::endl;

  EXPECT_EQ(total_zmp_val, Eigen::Vector2d::Zero());
}

TEST_F(TestLeggedZmp, TestSmallNormalForce) {
  // Parameters
  double expected_fz, computed_fz;
  Eigen::Vector2d expected_zmp;

  expected_fz = 0.0005;
  ASSERT_LE(expected_fz, normal_force_threshold);

  l_ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz/2.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  r_ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz/2.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());

  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();
  auto com = robot.GetModel()->EvalCenterOfMassPosition();

  auto total_zmp_val = robot.EvalZeroMomentPoint();
  std::cout << (total_zmp_val - com.segment(0, 2)).transpose() << std::endl;

  EXPECT_EQ(total_zmp_val, Eigen::Vector2d::Zero());
}

TEST_F(TestLeggedZmp, TestBigNormalForce) {
  // Parameters
  double expected_fz, computed_fz;
  Eigen::Vector2d expected_zmp;
  expected_zmp << 0.0, -5.0e-05;

  expected_fz = 10.0;
  l_ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz/2, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  r_ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz/2, 0.0, 0.0, 0.0, 0.0, 0.0).finished());

  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();
  auto com = robot.GetModel()->EvalCenterOfMassPosition();

  auto total_zmp_val = robot.EvalZeroMomentPoint();
  std::cout << (total_zmp_val - com.segment(0, 2)).transpose() << std::endl;

  EXPECT_LE((total_zmp_val - expected_zmp).norm(), tolerance);
}

TEST_F(TestLeggedZmp, TestBigNormalForceXY) {
  // Parameters
  double expected_fz, computed_fz;
  Eigen::Vector2d expected_zmp;

  expected_fz = 10.0;
  expected_zmp << -0.247050327241624, -0.607915794022337;
  l_ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz/2, 0.2, 0.5, 0.0, 2.0, 1.0).finished());
  r_ft_sensor->SetFakeWrench(
      (Vector6d() << -expected_fz/2, 0.1, 0.3, 0.0, 4.0, 1.5).finished());

  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();
  auto com = robot.GetModel()->EvalCenterOfMassPosition();

  auto total_zmp_val = robot.EvalZeroMomentPoint();
  std::cout << (total_zmp_val - com.segment(0, 2)).transpose() << std::endl;

  std::cout << total_zmp_val.transpose() << std::endl;

  EXPECT_LE((total_zmp_val - expected_zmp).norm(), tolerance);
}
