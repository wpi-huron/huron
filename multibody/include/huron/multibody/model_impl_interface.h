#pragma once

#include <vector>
#include <memory>

#include "huron/types.h"
#include "huron/control_interfaces/joint.h"
#include "joint_common.h"

namespace huron {
namespace multibody {
namespace internal {

class ModelImplInterface {
 public:
  ModelImplInterface() = default;
  ModelImplInterface(const ModelImplInterface&) = delete;
  ModelImplInterface& operator=(const ModelImplInterface&) = delete;
  virtual ~ModelImplInterface() = default;

  virtual void BuildFromUrdf(const std::string& urdf_path,
                             JointType root_joint_type);

  virtual const std::vector<std::string>& GetJointNames() const;
  virtual std::weak_ptr<Joint> GetJoint(const std::string& name) const;
  virtual std::weak_ptr<Joint> GetJoint(size_t joint_index) const;

  virtual JointType GetJointType(size_t joint_index) const;
  virtual JointIndex GetJointIndex(const std::string& joint_name) const = 0;

  virtual std::unique_ptr<JointDescription> GetJointDescription(
    JointIndex joint_index) const;
  virtual std::unique_ptr<JointDescription> GetJointDescription(
    const std::string& joint_name) const;

  virtual Eigen::Affine3d
  GetJointTransformInWorld(size_t joint_index) const;

  virtual FrameIndex GetFrameIndex(
    const std::string& frame_name) const;
  virtual const std::string& GetFrameName(FrameIndex frame_index) const;
  virtual FrameType GetFrameType(FrameIndex frame_index) const;
  virtual Eigen::Affine3d GetFrameTransform(FrameIndex from_frame,
                                            FrameIndex to_frame) const;
  virtual Eigen::Affine3d GetFrameTransformInWorld(FrameIndex frame) const;

  virtual Eigen::Vector3d EvalCenterOfMassPosition();
  virtual Eigen::Vector3d GetCenterOfMassPosition() const;

  virtual Eigen::VectorXd NeutralConfiguration() const;

  /**
   * @brief Get the generalized accelerations of the model.
   */
  virtual const Eigen::VectorXd& GetAccelerations() const;

  /**
   * @brief Get the joint torques.
   */
  virtual const Eigen::VectorXd& GetTorques() const;

  /**
   * @brief Get the mass matrix with the cached value.
   */
  virtual const Eigen::MatrixXd& GetMassMatrix() const;

  /**
   * @brief Get the Coriolis matrix with the cached value.
   */
  virtual const Eigen::MatrixXd& GetCoriolisMatrix() const;

  /**
   * @brief Get the nonlinear effects vector.
   */
  virtual const Eigen::VectorXd& GetNonlinearEffects() const;

  /**
   * @brief Get the gravity vector.
   */
  virtual const Eigen::VectorXd& GetGravity() const;

  /**
   * @brief Get the spatial momentum with respect to the specified frame.
   */
  virtual const huron::Vector6d& GetSpatialMomentum() const;

  /**
   * @brief Get the centroidal momentum.
   */
  virtual huron::Vector6d GetCentroidalMomentum() const;

  /**
   * @brief Get the centroidal momentum matrix with the cached value.
   */
  virtual const huron::Matrix6Xd& GetCentroidalMatrix() const;

  virtual void ComputeAll(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v);

  virtual void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q);
  virtual void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v);
  virtual void ForwardKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& a);

  virtual bool is_built() const;

  virtual size_t num_positions() const;
  virtual size_t num_velocities() const;
  virtual size_t num_joints() const;
  virtual size_t num_frames() const;

};

}  // namespace internal
}  // namespace multibody
}  // namespace huron
