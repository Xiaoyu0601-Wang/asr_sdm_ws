#include "asr_sdm_hardware/comm_protocol.hpp"

namespace amp
{
CommProtocol::CommProtocol()
{
}

CommProtocol::~CommProtocol()
{
}

bool CommProtocol::setActuatorCMD(
  std::vector<std::vector<float>> & actuator_cmd, std::vector<uint8_t> * msg)
{
  // Convert the first actuator command to a CAN message
  if (actuator_cmd.empty() || msg == nullptr) {
    return false;
  }

  // Clear the message vector
  msg->clear();

  // Assuming each actuator command has at least one element (unit_id)
  uint8_t unit_id = static_cast<uint8_t>(actuator_cmd[0][0]);
  msg->push_back(unit_id);

  // Add more data from actuator_cmd as needed
  for (size_t i = 1; i < actuator_cmd[0].size() && i < 8; ++i) {
    msg->push_back(static_cast<uint8_t>(actuator_cmd[0][i]));
  }

  // Pad the message to ensure it has exactly 8 bytes
  while (msg->size() < 8) {
    msg->push_back(0);
  }

  return true;
}

// void CommProtocol::interfaceSetup(void)
// {
//   std::vector<uint8_t> msg;
//   for (uint8_t i = 0; i < 8; i++) {
//     msg.push_back(i);
//   }
// }

// void CommProtocol::writeCANMsg(uint16_t screwUnitID, std::vector<uint8_t> & msg)
// {
// }

// void CommProtocol::readCANMsg(uint16_t screwUnitID, std::vector<uint8_t> * msg)
// {
// }

}  // namespace amp
