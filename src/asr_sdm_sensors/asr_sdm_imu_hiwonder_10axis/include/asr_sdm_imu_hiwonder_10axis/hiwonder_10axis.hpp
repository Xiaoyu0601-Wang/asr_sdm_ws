#ifndef HIWONDER_10AXIS_HPP_
#define HIWONDER_10AXIS_HPP_

#include <iostream>
#include <memory>
#include <vector>

/* ROS2 headers */
#include "data_structure/circular_queue.hpp"

#include <rclcpp/rclcpp.hpp>

/* c-periphery headers */
#include "periphery/serial.h"

#define FRAME_HEADER 0x50

namespace amp
{
class HiWonderIMU
{
public:
  HiWonderIMU();
  ~HiWonderIMU();

  void initModules(const std::string & uart_port, uint32_t uart_baudrate);
  void ImuLoopCallback();

private:
  enum {
    TYPE_ACCEL = 0x51,
    TYPE_GYRO = 0x52,
    TYPE_ANGLE = 0x53,
    // TYPE_MAG = 0x54,
    // TYPE_JOINT_TORQUE = 0x55,
    TYPE_ALTITUDE = 0x56,
  };

  bool handle_serial_data(uint8_t byte);
  std::vector<int16_t> hex_to_short(const std::vector<uint8_t> & data, size_t start);
  bool check_sum(const std::vector<uint8_t> & data, size_t check_index);
  void publish_imu();
  std::vector<double> get_quaternion_from_euler(double roll, double pitch, double yaw);

  serial_t * serial_;
  const char * uart_port_;
  CircularQueue<uint8_t> data_rxbuffer_;

  typedef std::shared_ptr<HiWonderIMU> Ptr;
};
}  // namespace amp

#endif /* HIWONDER_10AXIS_HPP_ */