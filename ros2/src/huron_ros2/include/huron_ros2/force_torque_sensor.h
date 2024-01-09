#pragma once

#include "huron/sensors/force_torque_sensor.h"

namespace huron {
namespace ros2 {

class HuronNode;

class ForceTorqueSensor : public huron::ForceTorqueSensor {
 public:
  ForceTorqueSensor(size_t index,
                    bool reverse_wrench_direction,
                    std::weak_ptr<const multibody::Frame> frame,
                    std::weak_ptr<const HuronNode> node);
  ForceTorqueSensor(const ForceTorqueSensor&) = delete;
  ForceTorqueSensor& operator=(const ForceTorqueSensor&) = delete;
  ~ForceTorqueSensor() override = default;

  void RequestStateUpdate() override;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

 protected:
  Vector6d DoGetWrenchRaw() override;

 private:
  size_t index_;
  std::weak_ptr<const HuronNode> node_;
};

}  // namespace ros2
}  // namespace huron
