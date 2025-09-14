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
  uint8_t reg, uint8_t operation, std::vector<std::vector<int32_t>> & cmd,
  std::vector<uint8_t> * msg)
{
  auto actuator_cmd = cmd;

  // Convert the first actuator command to a CAN message
  if (actuator_cmd.empty() || msg == nullptr) {
    return false;
  }

  // Clear the message vector
  msg->clear();

  for (size_t i = 0; i < actuator_cmd.size(); ++i) {
    // First element is unit_id
    msg->push_back((uint8_t)(actuator_cmd[i][0] >> 24));
    msg->push_back((uint8_t)(actuator_cmd[i][0] >> 16));
    msg->push_back((uint8_t)(actuator_cmd[i][0] >> 8));
    msg->push_back((uint8_t)(actuator_cmd[i][0] & 0xFF));

    msg->push_back(HEADER);
    msg->push_back(HEADER);
    msg->push_back(operation);
    msg->push_back(reg);
    if (reg == REGISTER_SCREW_VEL) {
      // std::cout << "REGISTER_SCREW_VEL" << std::endl;
      int16_t screw1_vel = static_cast<int16_t>(actuator_cmd[i][1]);
      msg->push_back(screw1_vel >> 8);
      msg->push_back(screw1_vel & 0xFF);
      int16_t screw2_vel = static_cast<int16_t>(actuator_cmd[i][2]);
      msg->push_back(screw2_vel >> 8);
      msg->push_back(screw2_vel & 0xFF);
    } else if (reg == REGISTER_JOINT_POS) {
      int16_t joint1_pos = static_cast<int16_t>(actuator_cmd[i][3]);
      msg->push_back(joint1_pos >> 8);
      msg->push_back(joint1_pos & 0xFF);
      int16_t joint2_pos = static_cast<int16_t>(actuator_cmd[i][4]);
      msg->push_back(joint2_pos >> 8);
      msg->push_back(joint2_pos & 0xFF);
    }

    // msg->push_back(HEADER);
    // msg->push_back(HEADER);
    // msg->push_back(WRITE);
    // msg->push_back(REGISTER_JOINT_POS);
    // int16_t joint1_pos = static_cast<int16_t>(actuator_cmd[i][3]);
    // msg->push_back(joint1_pos >> 8);
    // msg->push_back(joint1_pos & 0xFF);
    // int16_t joint2_pos = static_cast<int16_t>(actuator_cmd[i][4]);
    // msg->push_back(joint2_pos >> 8);
    // msg->push_back(joint2_pos & 0xFF);
  }

  return true;
}
// // Pad the message to ensure it has exactly 8 bytes
// while (msg->size() < 8) {
//   msg->push_back(0);
// }

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
