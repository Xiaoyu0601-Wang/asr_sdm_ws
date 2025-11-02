#include "asr_sdm_imu_hiwonder_10axis/imu_hiwonder_10axis.hpp"

namespace amp
{
ImuHiWonder10Axis::ImuHiWonder10Axis() : serial_(serial_new())
{
}

ImuHiWonder10Axis::~ImuHiWonder10Axis()
{
}

void ImuHiWonder10Axis::initModules(const std::string & uart_port, uint32_t uart_baudrate)
{
  uart_port_ = uart_port.c_str();

  /* Open /dev/ttyS* with baudrate, and defaults of 8N1, no flow control */
  if (serial_open(serial_, uart_port_, uart_baudrate) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("hardware"), "Serial port %s: %s", uart_port_, serial_errmsg(serial_));
  }
}

void ImuHiWonder10Axis::readImuData()
{
  // std::vector<uint8_t> buff;
  // while (running_) {
  //   if (serial_available(serial_)) {
  //     size_t n = serial_available(serial_);
  //     std::vector<uint8_t> data(n);
  //     serial_read(serial_, data.data(), n);
  //     for (size_t i = 0; i < n; ++i) {
  //       if (handle_serial_data(data[i])) {
  //         publish_imu();
  //       }
  //     }
  //   }
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }
}

}  // namespace amp