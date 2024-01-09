#include <gtest/gtest.h>
#include "huron/multibody/pinocchio_model_impl.h"

#ifdef HURON_USE_PINOCCHIO
class PinocchioModelImplTest : public testing::Test {
 protected:
  void SetUp() override {
    impl_rrbot.BuildFromUrdf("rrbot.urdf",
                             huron::multibody::JointType::kFixed);
    impl_huron.BuildFromUrdf("huron.urdf",
                             huron::multibody::JointType::kFreeFlyer);
  }

  huron::multibody::internal::PinocchioModelImpl impl_rrbot;
  huron::multibody::internal::PinocchioModelImpl impl_huron;
};

TEST_F(PinocchioModelImplTest, RRBotGeneralChecks) {
  auto v = impl_rrbot.GetJointNames();
  std::cout << "id\tname\t\tparent_frame\tchild_frame" << std::endl;
  for (auto& n : v) {
    std::unique_ptr<huron::multibody::JointDescription> jd =
      impl_rrbot.GetJointDescription(n);
    std::cout << *jd << std::endl;
  }
  EXPECT_EQ(impl_rrbot.GetJointIndex("universe"), 0);
  EXPECT_EQ(impl_rrbot.GetJointIndex("joint1"), 1);
  EXPECT_EQ(impl_rrbot.GetJointIndex("joint2"), 2);
  // non-fixed joints + universe
  EXPECT_EQ(impl_rrbot.num_joints(), 2 + 1);
  // non-fixed joinnts + universe + root_joint + fixed joints + links
  EXPECT_EQ(impl_rrbot.num_frames(), 2 + 1 + 1 + 1 + 4);
  EXPECT_EQ(impl_rrbot.num_positions(), 2);
  EXPECT_EQ(impl_rrbot.num_velocities(), 2);
}

TEST_F(PinocchioModelImplTest, HURONGeneralChecks) {
  EXPECT_EQ(impl_huron.GetJointIndex("universe"), 0);
  EXPECT_EQ(impl_huron.GetJointIndex("root_joint"), 1);
  EXPECT_EQ(impl_huron.GetJointIndex("l_hip_yaw_joint"), 2);
  EXPECT_EQ(impl_huron.GetJointIndex("l_hip_roll_joint"), 3);
  EXPECT_EQ(impl_huron.GetJointIndex("l_hip_pitch_joint"), 4);
  EXPECT_EQ(impl_huron.GetJointIndex("l_knee_pitch_joint"), 5);
  EXPECT_EQ(impl_huron.GetJointIndex("l_ankle_pitch_joint"), 6);
  EXPECT_EQ(impl_huron.GetJointIndex("l_ankle_roll_joint"), 7);
  EXPECT_EQ(impl_huron.GetJointIndex("r_hip_yaw_joint"), 8);
  EXPECT_EQ(impl_huron.GetJointIndex("r_hip_roll_joint"), 9);
  EXPECT_EQ(impl_huron.GetJointIndex("r_hip_pitch_joint"), 10);
  EXPECT_EQ(impl_huron.GetJointIndex("r_knee_pitch_joint"), 11);
  EXPECT_EQ(impl_huron.GetJointIndex("r_ankle_pitch_joint"), 12);
  EXPECT_EQ(impl_huron.GetJointIndex("r_ankle_roll_joint"), 13);
  // non-fixed joints + root_joint + universe
  EXPECT_EQ(impl_huron.num_joints(), 12 + 1 + 1);
  // non-fixed joinnts + universe + root_joint + fixed joints + links
  EXPECT_EQ(impl_huron.num_frames(), 12 + 1 + 1 + 4 + 17);
  EXPECT_EQ(impl_huron.num_positions(), 12 + 7);
  EXPECT_EQ(impl_huron.num_velocities(), 12 + 6);
}
#endif
