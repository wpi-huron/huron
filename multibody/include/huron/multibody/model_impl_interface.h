#pragma once

#include <eigen3/Eigen/Dense>
#include <string>
#include <memory>

#include "huron/types.h"
#include "huron/multibody/frame.h"
#include "huron/multibody/joint_common.h"
#include "huron/control_interfaces/joint.h"

namespace huron {
namespace multibody {

class ModelImplInterface {
 public:
  ModelImplInterface();
  ModelImplInterface(const ModelImplInterface&) = delete;
  ModelImplInterface& operator=(const ModelImplInterface&) = delete;
  virtual ~ModelImplInterface() = default;

  virtual void BuildFromUrdf(const std::string& urdf_path) = 0;

  virtual std::vector<std::string> GetJointNames() const = 0;
  virtual std::weak_ptr<Joint> GetJoint(const std::string& name) const = 0;
  virtual std::weak_ptr<Joint> GetJoint(size_t joint_index) const = 0;

  virtual std::unique_ptr<JointDescription> GetJointDescription(
    JointIndex joint_index) const = 0;
  virtual std::unique_ptr<JointDescription> GetJointDescriptionFromChildFrame(
    FrameIndex child_frame_index) const = 0;

  virtual const Eigen::Affine3d& GetJointTransformInWorld(size_t joint_index) const = 0;

  virtual const FrameIndex& GetFrameIndex(
    const std::string& frame_name) const = 0;
  virtual const Eigen::Affine3d& GetFrameTransform(FrameIndex from_frame,
                                            FrameIndex to_frame) const = 0;
  virtual const Eigen::Affine3d& GetFrameTransformInWorld(FrameIndex frame) const = 0;

  virtual const Eigen::VectorXd& NeutralConfiguration() const = 0;

  /**
   * @brief Get the generalized accelerations of the model.
   */
  virtual const Eigen::VectorXd& GetAccelerations() const = 0;

  /**
   * @brief Get the joint torques.
   */
  virtual const Eigen::VectorXd& GetTorques() const = 0;

  /**
   * @brief Get the mass matrix with the cached value.
   */
  virtual const Eigen::MatrixXd& GetMassMatrix() const = 0;

  /**
   * @brief Get the Coriolis matrix with the cached value.
   */
  virtual const Eigen::MatrixXd& GetCoriolisMatrix() const = 0;

  /**
   * @brief Get the nonlinear effects vector.
   */
  virtual const Eigen::VectorXd& GetNonlinearEffects() const = 0;

  /**
   * @brief Get the gravity vector.
   */
  virtual const Eigen::VectorXd& GetGravity() const = 0;

  /**
   * @brief Get the spatial momentum with respect to the specified frame.
   */
  virtual const huron::Vector6d& GetSpatialMomentum() const = 0;

  /**
   * @brief Get the centroidal momentum.
   */
  virtual const huron::Vector6d& GetCentroidalMomentum() const = 0;

  /**
   * @brief Get the centroidal momentum matrix with the cached value.
   */
  virtual const huron::Matrix6Xd& GetCentroidalMatrix() const = 0;

  virtual void ComputeAll(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v) = 0;

  virtual void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q) = 0;
  virtual void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v) = 0;
  virtual void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& a) = 0;

  size_t num_joints() const { return num_joints_; }
  size_t num_frames() const { return num_frames_; }
  size_t num_positions() const { return num_positions_; }
  size_t num_velocities() const { return num_velocities_; }

 protected:
  size_t num_joints_;
  size_t num_frames_;
  size_t num_positions_;
  size_t num_velocities_;

  virtual JointType GetJointType(size_t joint_index) const = 0;
};

}  // namespace multibody
}  // namespace huron
