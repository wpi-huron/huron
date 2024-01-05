#pragma once

#include "huron/multibody/model_impl_types.h"
#include "huron/multibody/model_impl_interface.h"
#include "huron/multibody/frame.h"

namespace huron {
namespace multibody {

class Model : public std::enable_shared_from_this<Model> {
  using ModelImplInterface = internal::ModelImplInterface;
 public:
  Model();
  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;
  ~Model() = default;

  /**
   * Adds a model implementation to the model.
   *
   * @param type The type of the model implementation.
   * @param set_as_default If true, the model implementation will be set as
   *                       the default model implementation.
   * @throws std::runtime_error if the model implementation is not available.
   */
  void AddModelImpl(ModelImplType type, bool set_as_default = false);

  /**
   * Adds a joint to the model.
   *
   * @param index The index of the joint.
   * @param args Arguments to be passed to the constructor of the joint.
   */
  template<typename ...Args>
  void AddJoint(JointIndex index,
                       Args&&... args) {
    assert(!is_constructed_);
    assert(!is_finalized_);
    if (joints_[index] != nullptr) {
      // TODO(dtbpkmte): provide index information in the error message.
      throw std::runtime_error("Joint already exists at this index.");
    }
    joints_[index] = std::make_shared<Joint>(std::forward<Args>(args)...);
  }

  Joint* const GetJoint(JointIndex index);
  Joint* const GetJoint(const std::string& name);

  void SetJointStateProvider(JointIndex index,
                             std::shared_ptr<StateProvider> state_provider);

  JointIndex GetJointIndex(const std::string& joint_name) const;
  /**
   * Adds a frame to the model.
   * Currently supported formats for external users:
   * - AddFrame<LogicalFrame>(name, parent_frame, transform_function)
   *
   * @note As of now, frames can only be added after the model is built from
   *      URDF. This will be changed in the future.
   *
   * @param index The index of the frame.
   * @param args Arguments to be passed to the constructor of the frame.
   */
  // template<typename FrameImpl, typename ...Args>
  // std::weak_ptr<const Frame> AddFrame(const std::string& name, Args&&... args);

  template<typename FrameImpl, typename ...Args>
  std::weak_ptr<const Frame> AddFrame(const std::string& name, Args&&... args) {
    static_assert(
      std::is_base_of_v<Frame, FrameImpl>,
      "Invalid frame type.");
    static_assert(
      std::is_base_of_v<enable_protected_make_shared<FrameImpl>, FrameImpl>,
      "Frame-derived class must also derive from enable_protected_make_shared.");
    assert(is_constructed_);
    assert(!is_finalized_);

    DoAddFrame<FrameImpl>(name, true, std::forward<Args>(args)...);
    return frames_.back();
  }

  std::weak_ptr<const Frame> GetFrame(FrameIndex index) const;
  std::weak_ptr<const Frame> GetFrame(const std::string& name) const;

  void BuildFromUrdf(const std::string& urdf_path,
                     JointType root_joint_type = JointType::kFixed);

  /**
   * Performs final configuration and checks the validity of the model:
   * - Checks if all joints are added to the model.
   * - Adjusts the joint state indices in the states vector.
   * This method also sets the initial state [q, v] of the model.
   * @throws std::runtime_error if the model is not valid.
   */
  void Finalize(const Eigen::VectorXd& initial_state);
  void Finalize();

  /**
   * Updates the joint states [q, v] of the model.
   */
  void UpdateJointStates();

  void SetDefaultModelImpl(size_t index) {
    default_impl_index_ = index;
  }
  size_t GetDefaultModelImpl() const {
    return default_impl_index_;
  }


  // Kinematics and Dynamics wrapper functions
  Eigen::Affine3d GetJointTransformInWorld(size_t joint_index) const;

  FrameIndex GetFrameIndex(const std::string& frame_name) const;
  const std::string& GetFrameName(FrameIndex frame_index) const;
  Eigen::Affine3d GetFrameTransform(FrameIndex from_frame,
                                    FrameIndex to_frame) const;
  Eigen::Affine3d GetFrameTransformInWorld(FrameIndex frame) const;

  Eigen::VectorXd NeutralConfiguration() const;

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
  huron::Vector6d GetCentroidalMomentum() const;

  /**
   * @brief Get the centroidal momentum matrix with the cached value.
   */
  const huron::Matrix6Xd& GetCentroidalMatrix() const;

  void ComputeAll();

  void ForwardKinematics();

  bool is_finalized() const {
    return is_finalized_;
  }

  size_t num_positions() const {
    return num_positions_;
  }

  size_t num_velocities() const {
    return num_velocities_;
  }

  size_t num_joints() const {
    return joints_.size();
  }

  size_t num_frames() const {
    return frames_.size();
  }

 protected:
  /**
   * Returns the model implementation at the given index.
   * This function is provided for testing subclasses only.
   */
  ModelImplInterface const * GetModelImpl(size_t index) const;

  // template<typename FrameImpl, typename ...Args>
  // void DoAddFrame(const std::string& name, bool is_user_defined,
  //                 Args&&... args);
  template<typename FrameImpl, typename ...Args>
  void DoAddFrame(const std::string& name, bool is_user_defined,
                          Args&&... args) {
    // Check if the frame name already exists
    if (frame_name_to_index_.find(name) != frame_name_to_index_.end()) {
      throw std::runtime_error("Frame name already exists.");
    }
    frames_.push_back(FrameImpl::make_shared(
                        frames_.size(),  // frame index
                        name,  // frame name
                        is_user_defined,
                        weak_from_this(),  // model
                        std::forward<Args>(args)...));
    frame_name_to_index_[name] = frames_.size() - 1;
  }

  void DoAddFrameFromModelDescription(FrameIndex idx,
                                      const std::string& name,
                                      FrameType type);

  size_t default_impl_index_ = 0;
  std::vector<std::unique_ptr<ModelImplInterface>> impls_;
  std::vector<std::shared_ptr<Joint>> joints_;
  /// \brief The joint states [q, v].
  Eigen::VectorXd states_;
  size_t num_positions_ = 0;
  size_t num_velocities_ = 0;

  /// \brief All frames, including those defined by the model description file
  /// and user-defined ones.
  std::vector<std::shared_ptr<Frame>> frames_;
  std::unordered_map<std::string, FrameIndex> frame_name_to_index_;

  bool is_constructed_ = false;
  bool is_finalized_ = false;
};

}  // namespace multibody
}  // namespace huron
