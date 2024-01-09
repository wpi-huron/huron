#include <iostream>
#include <chrono>
#include <thread>

#include "huron/utils/time.h"
#include "huron/driver/serial/wjwwood_serial.h"
#include "huron/sensors/force_sensing_resistor_array_serial.h"

int main(int argc, char* argv[]) {
  huron::ForceSensingResistorArraySerial fsr_array{
    "LFoot",
    4,
    std::make_shared<huron::driver::serial::Serial>(
      "/dev/ttyACM0",
      115200,
      huron::driver::serial::Parity::None,
      huron::driver::serial::StopBits::One,
      huron::driver::serial::FlowControl::None)};

  while (true) {
    std::cout << fsr_array.GetValues() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}
