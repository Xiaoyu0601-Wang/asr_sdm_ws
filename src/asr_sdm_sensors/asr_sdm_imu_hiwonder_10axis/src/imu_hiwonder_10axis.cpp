#include "asr_sdm_imu_hiwonder_10axis/imu_hiwonder_10axis.hpp"

namespace amp
{
ImuHiWonder10Axis::ImuHiWonder10Axis()
: serial_(serial_new()), data_rxbuffer_(FRAME_SIZE), buffer_cleared_(false)
{
}

ImuHiWonder10Axis::~ImuHiWonder10Axis()
{
  serial_close(serial_);
  serial_free(serial_);
}

bool ImuHiWonder10Axis::initModules(const std::string & uart_port, uint32_t uart_baudrate)
{
  uart_port_ = uart_port.c_str();

  /* Open /dev/ttyS* with baudrate, and defaults of 8N1, no flow control */
  if (serial_open(serial_, uart_port_, uart_baudrate) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("hardware"), "Serial port %s: %s", uart_port_, serial_errmsg(serial_));
    return false;
  }

  uint8_t buf[256];

  // Clear any leftover data in the serial buffer on the first call.
  // Drain until no more data (serial_read returns <= 0 on timeout/error)
  while (true) {
    int ret = serial_read(serial_, buf, sizeof(buf), 50);  // 50 ms timeout
    if (ret <= 0) break;
    // discard read bytes and loop until no more data
  }

  return true;
}

// helper: convert 4 consecutive little-endian int16 pairs starting at `start` to int16 values
std::array<int16_t, 4> ImuHiWonder10Axis::hex2short(std::array<uint8_t, 11> & data, size_t start)
{
  std::array<int16_t, 4> out{};
  for (size_t i = 0; i < 4; ++i) {
    size_t idx = start + 2 * i;
    out[i] = static_cast<int16_t>((data[idx] & 0xFF) | (data[idx + 1] << 8));
  }
  return out;
}

bool ImuHiWonder10Axis::checkSum(std::array<uint8_t, 11> & data, size_t check_index)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < check_index; ++i) sum = static_cast<uint8_t>(sum + data[i]);
  return (sum & 0xFF) == data[check_index];
}

bool ImuHiWonder10Axis::readDataLoop(
  std::array<double, 3> & acceleration, std::array<double, 3> & angularVelocity,
  std::array<double, 3> & angle_deg, std::array<double, 2> & pressure)
{
  uint8_t buf[256];

  // Clear any leftover data in the serial buffer on the first call.
  // Drain until no more data (serial_read returns <= 0 on timeout/error)
  while (true) {
    int ret = serial_read(serial_, buf, sizeof(buf), 10);  // 10 ms timeout
    if (ret <= 0) break;
    for (int i = 0; i < ret; i++) {
      data_rxbuffer_.push_overwrite(buf[i]);
    }
    // discard read bytes and loop until no more data
  }

  uint8_t temp = 0;
  std::array<uint8_t, 11> buff{};
  std::array<int16_t, 4> data;
  // Try to parse one complete frame from the rx buffer
  while (data_rxbuffer_.pop(temp)) {
    // look for header 0x55
    if (temp != 0x55) continue;

    data_rxbuffer_.pop(temp);
    // got header, now try to read the rest of the frame
    switch (temp) {
      case FrameType::ACCEL:
        for (size_t i = 0; i < 11; i++) {
          data_rxbuffer_.pop(temp);
          buff[i] = temp;
        }
        data = hex2short(buff, 2);
        for (int i = 0; i < 3; ++i) {
          // accelerometer: scale from device units to m/s^2
          acceleration[i] = static_cast<double>(data[i]) / 32768.0 * 16.0 * 9.8;
        }
        break;
      case FrameType::GYRO:
        for (size_t i = 0; i < 11; i++) {
          data_rxbuffer_.pop(temp);
          buff[i] = temp;
        }
        data = hex2short(buff, 2);
        for (int i = 0; i < 3; ++i) {
          // gyroscope: deg/s -> convert to rad/s
          angularVelocity[i] = static_cast<double>(data[i]) / 32768.0 * 2000.0 * M_PI / 180.0;
        }
        break;
      case FrameType::ANGLE:
        for (size_t i = 0; i < 11; i++) {
          data_rxbuffer_.pop(temp);
          buff[i] = temp;
        }
        data = hex2short(buff, 2);
        for (int i = 0; i < 3; ++i) {
          angle_deg[i] = static_cast<double>(data[i]) / 32768.0 * 180.0;
        }
        break;
      case FrameType::ALTITUDE:
        for (size_t i = 0; i < 11; i++) {
          data_rxbuffer_.pop(temp);
          buff[i] = temp;
        }
        data = hex2short(buff, 2);
        for (int i = 0; i < 2; ++i) {
          pressure[i] = static_cast<double>(data[i]) / 32768.0 * 180.0;
        }
        break;
    }
  }

  // If we reach here, we have successfully parsed one complete frame
  return true;
}

}  // namespace amp