#include <iostream>
#include <sockcanpp/CanDriver.hpp>
#include "huron/odrive/odrive_can.h"


int main(int argc, char* argv[]) {
  can_Message_t msg;
  msg.id = 0 << huron::odrive::ODriveCAN::NUM_CMD_ID_BITS;
  msg.id += huron::odrive::ODriveCAN::MSG_GET_ENCODER_COUNT;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;

  uint32_t id = msg.id;
  if (msg.rtr) {
    id |= 0x800;
  }
  std::cout << "id: " << id << std::endl;
  sockcanpp::CanId canid{id};
  std::cout << "Standard? " << canid.isStandardFrameId() << std::endl;
  std::cout << "Extended? " << canid.isExtendedFrameId() << std::endl;
  std::cout << "RTR: " << canid.hasRtrFrameFlag() << std::endl;
  return 0;
}
