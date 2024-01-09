#include "huron/control_interfaces/sensor_with_frame.h"

namespace huron {

SensorWithFrame::SensorWithFrame(const Eigen::Vector2i& dim,
                                 std::weak_ptr<const multibody::Frame> frame)
  : Sensor(dim),
    frame_(std::move(frame)) {}

SensorWithFrame::SensorWithFrame(const Eigen::Vector2i& dim,
                                 std::weak_ptr<const multibody::Frame> frame,
                                 std::unique_ptr<Configuration> config)
  : Sensor(dim, std::move(config)), frame_(std::move(frame)) {}

SensorWithFrame::SensorWithFrame(int rows, int cols,
                                 std::weak_ptr<const multibody::Frame> frame)
  : Sensor(rows, cols),
    frame_(std::move(frame)) {}

SensorWithFrame::SensorWithFrame(int rows, int cols,
                                 std::weak_ptr<const multibody::Frame> frame,
                                 std::unique_ptr<Configuration> config)
  : Sensor(rows, cols, std::move(config)), frame_(std::move(frame)) {}

}  // namespace huron
