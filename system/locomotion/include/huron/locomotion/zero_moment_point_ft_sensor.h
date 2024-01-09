#pragma once

#include "huron/locomotion/zero_moment_point.h"

namespace huron {

class ZeroMomentPointFTSensor : public ZeroMomentPoint {
 public:
  ZeroMomentPointFTSensor(
    std::weak_ptr<const multibody::Frame> zmp_frame,
    double normal_force_threshold,
    const std::vector<std::shared_ptr<ForceTorqueSensor>>& ft_sensors);
  ZeroMomentPointFTSensor(const ZeroMomentPointFTSensor&) = delete;
  ZeroMomentPointFTSensor& operator=(const ZeroMomentPointFTSensor&) = delete;
  ~ZeroMomentPointFTSensor() override = default;

  Eigen::Vector2d Eval(double& fz) override;

 private:
  std::vector<std::shared_ptr<ForceTorqueSensor>> ft_sensors_;
};

}  // namespace huron
