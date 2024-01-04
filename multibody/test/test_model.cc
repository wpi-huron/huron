#include <gtest/gtest.h>
#include "huron/control_interfaces/robot.h"
#include "huron/multibody/model.h"
#include "huron/control_interfaces/constant_state_provider.h"

using namespace huron;  //NOLINT

class TestRobot : public Robot {
 public:
  TestRobot() : Robot()  {}
  ~TestRobot() override = default;

  bool Move(const std::vector<double>& values) override {return true;}
  bool Move(const Eigen::VectorXd& values) override {return true;}
  bool Stop() override {return true;}

  void Initialize() override {}
  void SetUp() override {}
  void Terminate() override {}
};

#ifdef HURON_USE_PINOCCHIO
class TestModelPinocchio : public testing::Test {
 protected:
  void SetUp() override {
    robot.GetModel()->AddModelImpl(
      huron::multibody::ModelImplType::kPinocchio, true);
    robot.GetModel()->BuildFromUrdf("rrbot.urdf");
    robot.GetModel()->SetJointStateProvider(
        1,
        std::make_shared<ConstantStateProvider>(
          Eigen::Vector2d::Zero()));
    robot.GetModel()->SetJointStateProvider(
        2,
        std::make_shared<ConstantStateProvider>(
          Eigen::Vector2d::Zero()));
    robot.GetModel()->Finalize();
  }

  // void TearDown() override {}

  TestRobot robot;
};

TEST_F(TestModelPinocchio, TestGeneral) {
  EXPECT_EQ(robot.GetModel()->num_joints(), 3);
  EXPECT_EQ(robot.GetModel()->num_frames(), 9);
  EXPECT_EQ(robot.GetModel()->num_positions(), 2);
  EXPECT_EQ(robot.GetModel()->num_velocities(), 2);
}

TEST_F(TestModelPinocchio, TestKinematics) {
  auto base_link_frame = robot.GetModel()->GetFrame("base_link");
  auto link1_frame = robot.GetModel()->GetFrame("link1");
  auto link2_frame = robot.GetModel()->GetFrame("link2");

  Eigen::Affine3d expected_base_link_pose(Eigen::Affine3d::Identity());
  expected_base_link_pose.translate(Eigen::Vector3d(0, 0, 1.0));
  Eigen::Affine3d expected_link1_pose =
    expected_base_link_pose * Eigen::Translation3d(0, 0.1, 1.95);
  Eigen::Affine3d expected_link2_pose =
    expected_link1_pose * Eigen::Translation3d(0, 0.1, 0.9);

  robot.GetModel()->UpdateStates();
  robot.GetModel()->ForwardKinematics();
  Eigen::Affine3d base_link_pose =
    base_link_frame.lock()->GetTransformInWorld();
  Eigen::Affine3d link1_pose =
    link1_frame.lock()->GetTransformInWorld();
  Eigen::Affine3d link2_pose =
    link2_frame.lock()->GetTransformInWorld();
  EXPECT_EQ(base_link_pose.matrix(),
            expected_base_link_pose.matrix());
  EXPECT_EQ(link1_pose.matrix(),
            expected_link1_pose.matrix());
  EXPECT_EQ(link2_pose.matrix(),
            expected_link2_pose.matrix());
}

#else
TEST(TestModelPinocchio, TestDepNotExist) {
  Robot robot;
  EXPECT_THROW(robot.GetModel()->AddModel(ModelImplType::kPinocchio),
               std::runtime_error));
}
#endif // HURON_USE_PINOCCHIO
