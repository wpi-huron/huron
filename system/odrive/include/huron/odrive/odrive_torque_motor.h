#pragma once

#include <memory>
#include <vector>

#include "huron/control_interfaces/torque_motor.h"
#include "huron/odrive/odrive_can.h"

namespace huron {
namespace odrive {

class TorqueMotor : public huron::TorqueMotor {
 public:
  TorqueMotor(
    std::unique_ptr<TorqueMotorConfiguration> config,
    std::shared_ptr<ODrive> odrive,
    double gear_ratio);
  TorqueMotor(
    std::shared_ptr<ODrive> odrive,
    double gear_ratio);
  explicit TorqueMotor(std::shared_ptr<ODrive> odrive);
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  ~TorqueMotor() = default;

  // GenericComponent methods
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  bool Move(double value) override;
  bool Move(const std::vector<double>& values) override;
  bool Move(const Eigen::VectorXd& values) override;
  bool Stop() override;

 private:
  std::shared_ptr<ODrive> odrive_;
};

}  // namespace odrive
}  // namespace huron
