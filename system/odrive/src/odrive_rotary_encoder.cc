#include "huron/odrive/odrive_rotary_encoder.h"

namespace huron {
namespace odrive {

ODriveEncoder::ODriveEncoder(std::unique_ptr<RotaryEncoderConfiguration> config,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder(std::move(config)), odrive_(std::move(odrive)) {}

ODriveEncoder::ODriveEncoder(double cpr,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder(cpr), odrive_(std::move(odrive)) {}


void ODriveEncoder::Initialize() {
}

void ODriveEncoder::SetUp() {
}

void ODriveEncoder::Terminate() {
}


void ODriveEncoder::DoUpdateState() {
  float pos, vel;
  odrive_->GetEncoderEstimates(pos, vel);
  count_ = pos * cpr_;
  velocity_ = vel * cpr_;
}

}  // namespace odrive
}  // namespace huron
