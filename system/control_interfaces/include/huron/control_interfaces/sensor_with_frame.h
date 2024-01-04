#pragma once

#include <eigen3/Eigen/Core>
#include "huron/control_interfaces/sensor.h"
#include "huron/multibody/frame.h"

namespace huron {

class SensorWithFrame : public Sensor {
  using Frame = multibody::Frame;
 public:
  SensorWithFrame(const Eigen::Vector2i& dim,
                  std::weak_ptr<const Frame> frame);
  SensorWithFrame(const Eigen::Vector2i& dim,
                  std::weak_ptr<const Frame> frame,
                  std::unique_ptr<Configuration> config);
  SensorWithFrame(int rows, int cols,
                  std::weak_ptr<const Frame> frame);
  SensorWithFrame(int rows, int cols,
                  std::weak_ptr<const Frame> frame,
                  std::unique_ptr<Configuration> config);
  SensorWithFrame(const SensorWithFrame&) = delete;
  SensorWithFrame& operator=(const SensorWithFrame&) = delete;
  ~SensorWithFrame() override = default;

  /**
   * @brief Get the sensor frame.
   */
  std::weak_ptr<const Frame> GetSensorFrame() const {
    return frame_;
  }

 private:
  std::weak_ptr<const Frame> frame_;
};

}  // namespace huron
