#pragma once

#include "model_impl_interface.h"
#include "model_impl_base.h"

namespace huron {
namespace multibody {

class Model {
  using ModelImplBase = internal::ModelImplBase;

 public:
  Model();
  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;
  ~Model() = default;

  /**
   * Adds a model implementation to the model.
   * The model implementation must be a subclass of ModelImplInterface.
   *
   * @param set_as_default If true, the model implementation will be set as
   *                       the default model implementation.
   * @param args Arguments to be passed to the constructor of the model
   *             implementation.
   */
  template<typename ImplType, typename ...Args>
  void AddModelImpl(bool set_as_default = false,
                    Args&&... args);

  /**
   * Returns the model implementation at the given index.
   */
  ModelImplInterface const * GetModelImpl(size_t index) const;

  /**
   * Adds a joint to the model.
   *
   * @param index The index of the joint.
   * @param args Arguments to be passed to the constructor of the joint.
   */
  template<typename ...Args>
  void AddJoint(JointIndex index, Args&&... args);

  Joint* const GetJoint(JointIndex index);

  void BuildFromUrdf(const std::string& urdf_path);

  /**
   * Performs final configuration and checks the validity of the model:
   * - Checks if all joints are added to the model.
   * - Adjusts the joint state indices in the states vector.
   * @throws std::runtime_error if the model is not valid.
   */
  void Finalize();

  /**
   * Updates the joint states [q, v] of the model.
   */
  void UpdateStates();

  void SetDefaultModelImpl(size_t index) {
    default_impl_index_ = index;
  }
  size_t GetDefaultModelImpl() const {
    return default_impl_index_;
  }

  // Kinematics and Dynamics functions
  const Eigen::Affine3d& GetJointPoseInWorld(size_t joint_index) const;

  const FrameIndex& GetFrameIndex(const std::string& frame_name) const;
  const Eigen::Affine3d& GetFrameTransform(FrameIndex from_frame,
                                    FrameIndex to_frame) const;
  const Eigen::Affine3d& GetFrameTransformInWorld(FrameIndex frame) const;

  const Eigen::VectorXd& NeutralConfiguration() const;

  /**
   * @brief Get the generalized accelerations of the model.
   */
  const Eigen::VectorXd& GetAccelerations() const;

  /**
   * @brief Get the joint torques.
   */
  const Eigen::VectorXd& GetTorques() const;

  /**
   * @brief Get the mass matrix with the cached value.
   */
  const Eigen::MatrixXd& GetMassMatrix() const;

  /**
   * @brief Get the Coriolis matrix with the cached value.
   */
  const Eigen::MatrixXd& GetCoriolisMatrix() const;

  /**
   * @brief Get the nonlinear effects vector.
   */
  const Eigen::VectorXd& GetNonlinearEffects() const;

  /**
   * @brief Get the gravity vector.
   */
  const Eigen::VectorXd& GetGravity() const;

  /**
   * @brief Get the spatial momentum with respect to the specified frame.
   */
  const huron::Vector6d& GetSpatialMomentum() const;

  /**
   * @brief Get the centroidal momentum.
   */
  const huron::Vector6d& GetCentroidalMomentum() const;

  /**
   * @brief Get the centroidal momentum matrix with the cached value.
   */
  const huron::Matrix6Xd& GetCentroidalMatrix() const;

  void ComputeAll();

  void ForwardKinematics();

 private:
  size_t default_impl_index_ = 0;
  std::vector<std::unique_ptr<ModelImplBase>> impls_;
  std::vector<std::shared_ptr<Joint>> joints_;
  Eigen::VectorXd states_;
  size_t num_positions_ = 0;
  size_t num_velocities_ = 0;

  bool is_constructed_ = false;
};

}  // namespace multibody
}  // namespace huron
