#include "huron/odrive/odrive_rotary_encoder.h"
namespace huron {
namespace odrive {

ODriveEncoder::ODriveEncoder(std::unique_ptr<RotaryEncoderConfiguration> config,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder(std::move(config)), odrive_(std::move(odrive)) {}
ODriveEncoder::ODriveEncoder(float cpr,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder(cpr), odrive_(std::move(odrive)) {}


void ODriveEncoder::Initialize() {
}

void ODriveEncoder::SetUp() {
}

void ODriveEncoder::Terminate() {
}


float ODriveEncoder::GetCount() {
  prev_count_ = count_;
  float pos, vel;
  odrive_->GetEncoderEstimates(pos, vel);
  count_ = pos * cpr_;
  velocity_ = vel * cpr_;
  return count_;
}

float ODriveEncoder::GetVelocityCount() {
  GetPosition();
  return velocity_;
}

}  // namespace odrive
}  // namespace huron
