#ifndef IMU_HIWONDER_10AXIS_HPP_
#define IMU_HIWONDER_10AXIS_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>

/* ROS2 headers */
#include "data_structure/circular_queue.hpp"

#include <rclcpp/rclcpp.hpp>

/* c-periphery headers */
#include "periphery/serial.h"

#define MAX_BUFFER_SIZE 1024
#define FRAME_HEADER 0x55
#define FRAME_SIZE 88

namespace amp
{
class ImuHiWonder10Axis
{
public:
  ImuHiWonder10Axis();
  ~ImuHiWonder10Axis();

  bool initModules(const std::string & uart_port, uint32_t uart_baudrate);
  bool readDataLoop(
    std::array<double, 3> & acceleration, std::array<double, 3> & angularVelocity,
    std::array<double, 3> & angle_deg, std::array<double, 2> & pressure);

private:
  // enum FrameType : uint8_t {
  //   TYPE_ACCEL = 0x51,
  //   TYPE_GYRO = 0x52,
  //   TYPE_ANGLE = 0x53,
  //   // TYPE_MAG = 0x54,
  //   // TYPE_JOINT_TORQUE = 0x55,
  //   TYPE_ALTITUDE = 0x56,
  // };
  enum FrameType : uint8_t { ACCEL = 0x51, GYRO = 0x52, ANGLE = 0x53, ALTITUDE = 0x56 };

  std::array<int16_t, 4> hex2short(std::array<uint8_t, 11> & data, size_t start);
  bool checkSum(std::array<uint8_t, 11> & data, size_t check_index);
  // std::vector<double> get_quaternion_from_euler(double roll, double pitch, double yaw);

  serial_t * serial_;
  const char * uart_port_;
  CircularQueue<uint8_t> data_rxbuffer_;

  bool buffer_cleared_;

  // typedef std::unique_ptr<ImuHiWonder10Axis> Ptr;
};
}  // namespace amp

#endif /* IMU_HIWONDER_10AXIS_HPP_ */