#pragma once

#include <memory>
#include "huron/control_interfaces/rotary_encoder.h"
#include "huron/odrive/odrive.h"

namespace huron {
namespace odrive {

class ODriveEncoder final : public huron::RotaryEncoder {
 public:
  ODriveEncoder(std::unique_ptr<RotaryEncoderConfiguration> config,
		std::shared_ptr<ODrive> odrive);
  ODriveEncoder(double cpr, std::shared_ptr<ODrive> odrive);
  ODriveEncoder(const ODriveEncoder&) = delete;
  ODriveEncoder& operator=(const ODriveEncoder&) = delete;
  ~ODriveEncoder() override = default;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

 protected:
  void DoUpdateState() override;

 private:
  std::shared_ptr<ODrive> odrive_;
};

}  // namespace odrive
}  // namespace huron
