#pragma once

#include <memory>
#include "huron/control_interfaces/rotary_encoder.h"
#include "huron/odrive/odrive.h"

namespace huron {
namespace odrive {

class ODriveEncoder : public huron::RotaryEncoder {
 public:
  explicit ODriveEncoder(float cpr, std::shared_ptr<ODrive> odrive);
  ODriveEncoder(const ODriveEncoder&) = delete;
  ODriveEncoder& operator=(const ODriveEncoder&) = delete;
  ~ODriveEncoder() override = default;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  float GetCount() override;
  float GetVelocityCount() override;

 protected:
  void ConfigureKey(std::string config_key, std::any config_value) override;

 private:
  std::shared_ptr<ODrive> odrive_;
};

}  // namespace odrive
}  // namespace huron
