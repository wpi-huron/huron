#pragma once

#include "huron/locomotion/zero_moment_point.h"

namespace huron {

class ZeroMomentPointFSRArray : public ZeroMomentPoint {
 public:
  ZeroMomentPointFSRArray(
    std::weak_ptr<const multibody::Frame> zmp_frame,
    double normal_force_threshold,
    std::shared_ptr<ForceSensingResistorArray> fsr_array);
  ZeroMomentPointFSRArray(const ZeroMomentPointFSRArray&) = delete;
  ZeroMomentPointFSRArray& operator=(const ZeroMomentPointFSRArray&) = delete;
  ~ZeroMomentPointFSRArray() override = default;

  Eigen::Vector2d Eval(double& fz) override;

 private:
  std::shared_ptr<ForceSensingResistorArray> fsr_array_;
};

}  // namespace huron
