#pragma once

#include <memory>
#include <eigen3/Eigen/Core>

#include "huron/sensors/force_torque.h"
#include "huron/sensors/force_sensing_resistor_array.h"

namespace huron {

class ZeroMomentPointFTSensor;
class ZeroMomentPointFSRArray;

class ZeroMomentPoint {
 public:
  static std::unique_ptr<ZeroMomentPointFTSensor> FromFTSensor(
    std::shared_ptr<ForceTorqueSensor> ft_sensor);
  static std::unique_ptr<ZeroMomentPointFSRArray> FromFSRArray(
    std::shared_ptr<ForceSensingResistorArray> fsr_array);

  ZeroMomentPoint(std::string frame, double normal_force_threshold);

  virtual void Compute(Eigen::Ref<Eigen::Vector2d> zmp, double& fz) = 0;
  inline void Compute(Eigen::Ref<Eigen::Vector2d> zmp) {
    double fz;
    Compute(zmp, fz);
  }
  inline Eigen::Vector2d Compute() {
    Eigen::Vector2d zmp;
    Compute(zmp);
    return zmp;
  }

  inline std::string GetFrame() const {
    return frame_;
  }

 protected:
  std::string frame_;
  double normal_force_threshold_;
};

class ZeroMomentPointFTSensor : public ZeroMomentPoint {
 public:
  ZeroMomentPointFTSensor(
    std::string frame,
    double normal_force_threshold,
    const Eigen::Vector3d& sensor_position,
    const Eigen::Vector3d& sensor_frame_zyx,
    std::shared_ptr<ForceTorqueSensor> ft_sensor);

  void Compute(Eigen::Ref<Eigen::Vector2d> zmp, double& fz) override;

 private:
  const Eigen::Vector3d sensor_position_;
  const Eigen::Vector3d sensor_frame_zyx_;
  std::shared_ptr<ForceTorqueSensor> ft_sensor_;
};

class ZeroMomentPointFSRArray : public ZeroMomentPoint {
 public:
  ZeroMomentPointFSRArray(
    std::string frame,
    double normal_force_threshold,
    const Eigen::VectorXd& sensor_x_positions,
    const Eigen::VectorXd& sensor_y_positions,
    std::shared_ptr<ForceSensingResistorArray> fsr_array);

  void Compute(Eigen::Ref<Eigen::Vector2d> zmp, double& fz) override;

 private:
  const Eigen::VectorXd sensor_x_positions_;
  const Eigen::VectorXd sensor_y_positions_;
  std::shared_ptr<ForceSensingResistorArray> fsr_array_;
};

class ZeroMomentPointTotal : public ZeroMomentPoint {
 public:
  ZeroMomentPointTotal(
    std::string frame,
    std::vector<std::shared_ptr<ZeroMomentPoint>> zmp_vector);
    
  void Compute(Eigen::Ref<Eigen::Vector2d> zmp, double& fz) override;

 private:
  std::vector<std::shared_ptr<ZeroMomentPoint>> zmp_vector_;
};

}  // namespace huron
