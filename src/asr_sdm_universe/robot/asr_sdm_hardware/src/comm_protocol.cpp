#include "asr_sdm_hardware/comm_protocol.hpp"

namespace amp
{
CommProtocol::CommProtocol()
{
}

CommProtocol::~CommProtocol()
{
}

void CommProtocol::setActuator(
  std::vector<std::vector<float>> & actuator_cmd, std::vector<uint8_t> & msg)
{
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
