#ifndef COMM_PROTOCOL_HPP_
#define COMM_PROTOCOL_HPP_

#include "asr_sdm_hardware/uart2can.hpp"

#include "asr_sdm_hardware_msgs/msg/hardware_cmd.hpp"

#include <iostream>
#include <memory>
#include <vector>

namespace amp
{

class CommProtocol
{
public:
  enum {
    HEADER = 0xCC,  // Header byte
    READ = 0x02,    // Read command
    WRITE = 0x03,   // Write command
  };

  // Register
  enum {
    // MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
    // MSG_ODRIVE_HEARTBEAT,
    // MSG_ODRIVE_ESTOP,
    REGISTER_HEARTBEAT = 0x00,  // Errors
    REGISTER_SCREW_VEL = 0x01,
    REGISTER_JOINT_POS = 0x02,
    REGISTER_JOINT_VEL = 0x03,
    REGISTER_JOINT_TORQUE = 0x04,
    REGISTER_IMU_RAW = 0x05,
    // MSG_GET_ENCODER_ERROR,
    // MSG_GET_SENSORLESS_ERROR,
    // MSG_SET_AXIS_NODE_ID,
    // MSG_SET_AXIS_REQUESTED_STATE,
    // MSG_SET_AXIS_STARTUP_CONFIG,
    // MSG_GET_ENCODER_ESTIMATES,
    // MSG_GET_ENCODER_COUNT,
    // MSG_SET_CONTROLLER_MODES,
    // MSG_SET_INPUT_POS,
    // MSG_SET_INPUT_VEL,
    // MSG_SET_INPUT_TORQUE,
    // MSG_SET_LIMITS,
    // MSG_START_ANTICOGGING,
    // MSG_SET_TRAJ_VEL_LIMIT,
    // MSG_SET_TRAJ_ACCEL_LIMITS,
    // MSG_SET_TRAJ_INERTIA,
    // MSG_GET_IQ,
    // MSG_GET_SENSORLESS_ESTIMATES,
    // MSG_RESET_ODRIVE,
    // MSG_GET_BUS_VOLTAGE_CURRENT,
    // MSG_CLEAR_ERRORS,
    // MSG_SET_LINEAR_COUNT,
    // MSG_SET_POS_GAIN,
    // MSG_SET_VEL_GAINS,
    // MSG_GET_ADC_VOLTAGE,
    // MSG_GET_CONTROLLER_ERROR,
    // MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
  };

  CommProtocol(amp::UART2CAN::Ptr device);
  ~CommProtocol();

  bool getIMU(std::vector<uint8_t> * msg);
  bool setActuatorCMD(
    uint8_t reg, uint8_t operation, std::vector<std::vector<int32_t>> & actuator_cmd,
    std::vector<uint8_t> * msg);

  // bool convertProtocol(std::vector<std::vector<float>> & msg, std::vector<uint8_t> *
  // msg_converted); void writeCANMsg(uint16_t screwUnitID, std::vector<uint8_t> & msg); void
  // readCANMsg(uint16_t screwUnitID, std::vector<uint8_t> * msg);
private:
  UART2CAN::Ptr device_;
};

}  // namespace amp

#endif /* COMM_PROTOCOL_HPP_ */
