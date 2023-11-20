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

  virtual Eigen::Vector2d Compute() = 0;

 protected:
  std::string frame;
};

class ZeroMomentPointFTSensor : public ZeroMomentPoint {
 public:
  Eigen::Vector2d Compute() override;

 private:
  Eigen::Vector3d sensor_position;
  std::shared_ptr<ForceTorqueSensor> ft_sensor;
};

class ZeroMomentPointFSRArray : public ZeroMomentPoint {
 public:
  Eigen::Vector2d Compute() override;

 private:
  Eigen::MatrixX3d sensor_positions;
  std::shared_ptr<ForceSensingResistorArray> fsr_array;
};

class ZeroMomentPointTotal : public ZeroMomentPoint {
 public:
  Eigen::Vector2d Compute() override;

 private:
  Eigen::MatrixX3d feet_positions;
  std::vector<std::shared_ptr<ZeroMomentPoint>> zmp_vector;
};

}  // namespace huron
