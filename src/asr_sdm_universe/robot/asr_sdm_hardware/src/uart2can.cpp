#include "asr_sdm_hardware/uart2can.hpp"

#include <iomanip>
#include <sstream>
#include <vector>

namespace amp
{

UART2CAN::UART2CAN()
: serial_(serial_new()), data_buffer_(MAX_FRAME_DATA_LENGTH)  // initialize with desired capacity

{
}

void UART2CAN::initModules(
  const std::string & uart_port, uint32_t uart_baudrate, uint32_t can_id_mask,
  uint8_t can_frame_length, uint8_t uart_frame_head, uint8_t uart_frame_tail)
{
  uart_port_ = uart_port.c_str();
  // serial_ = serial_new();

  /* Open /dev/ttyS3 with baudrate 115200, and defaults of 8N1, no flow control */
  if (serial_open(serial_, uart_port_, uart_baudrate) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "serial_ttyS3_open(): %s", serial_errmsg(serial_));
  }

  uart2can_frame_.can_id_mask = can_id_mask;
  uart2can_frame_.can_dlc = can_frame_length;
  uart2can_frame_.frame_head = uart_frame_head;
  uart2can_frame_.frame_tail = uart_frame_tail;
}

UART2CAN::~UART2CAN()
{
  serial_close(serial_);
  serial_free(serial_);
}

void UART2CAN::printVector(uint8_t * vec, uint8_t len, const std::string & name)
{
  std::ostringstream oss;
  oss << name << ": [";
  for (uint8_t i = 0; i < len; ++i) {
    oss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)vec[i];
    if (i != len - 1) oss << ", ";
  }
  oss << "]";
  RCLCPP_INFO(rclcpp::get_logger("uart2can"), "%s", oss.str().c_str());
}

/**
 * @name uartTransfer
 * @brief Brief description of what the function does.
 *
 * @param byte_number Description of the first parameter.
 * @param tx_buf Description of the second parameter.
 * @return Return type and what it represents.
 *
 * @note Any additional notes about usage.
 * @warning Warnings or important considerations.
 * @throws Exception types the function may throw.
 */
bool UART2CAN::uartTransfer(uint8_t byte_number)
{
  bool check_flag = false;

  if (byte_number < 4) {
    RCLCPP_WARN(rclcpp::get_logger("uart2can"), "Invalid byte number");
    return check_flag;
  }

  std::vector<uint8_t> tx_buf(byte_number);
  uint8_t popped_val;

  while (data_buffer_.pop(popped_val)) {
    // std::cout << "popped_val: 0x" << std::hex << std::setw(2) << std::setfill('0')
    //           << static_cast<int>(popped_val) << std::endl;
    // std::cout << "size=" << data_buffer_.size() << " (cap=" << data_buffer_.capacity() << ")"
    //           << std::endl;
    // std::cout << "uart2can_frame_.frame_head: 0x" << std::hex << std::setw(2) <<
    // std::setfill('0')
    //           << static_cast<int>(uart2can_frame_.frame_head) << std::endl;
    if (popped_val == uart2can_frame_.frame_head) {
      tx_buf[0] = uart2can_frame_.frame_head;
      for (uint8_t i = 1; i < uart2can_frame_.can_dlc + 7; i++) {
        data_buffer_.pop(popped_val);
        tx_buf[i] = popped_val;
      }

      /* Check for frame tail */
      data_buffer_.pop(popped_val);
      if (popped_val == uart2can_frame_.frame_tail) {
        tx_buf[byte_number - 1] = uart2can_frame_.frame_tail;
      } else {
        continue;
      }
      /* Write to the serial port */
      // printVector(tx_buf.data(), byte_number, "uart Data to send2");
      if (serial_write(serial_, tx_buf.data(), byte_number) < 0) {
        RCLCPP_INFO(rclcpp::get_logger("uart2can"), "serial_write(): %s\n", serial_errmsg(serial_));
      } else {
        check_flag = true;
        // RCLCPP_INFO(
        //   rclcpp::get_logger("uart2can"), "serial_write(): %d bytes written\n", byte_number);
      }
    }
  }

  return check_flag;
}

/*********************************************************************************************************
** Function name:           uart_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
// uint8_t UART2CAN::uart_configRate(const uint8_t canSpeed, const uint8_t canClock)
// {
//   return 0;
// }

/*********************************************************************************************************
** Function name:           uart_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
// void UART2CAN::uart_write_id(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
// {
// }

/*********************************************************************************************************
** Function name:           uart_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
// void UART2CAN::uart_read_id(const uint8_t mcp_addr, uint8_t * ext, uint32_t * id)
// {
// }

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
bool UART2CAN::setMsg(uint8_t * buf, uint8_t rtr, uint8_t ext, uint8_t len)
{
  // printVector(buf, len, "CAN Data to send");
  // uart2can_frame_.can_id = uart2can_frame_.can_id_mask | can_id;
  uart2can_frame_.m_nRtr = rtr;
  uart2can_frame_.can_ext_flag = ext;
  uart2can_frame_.can_dlc = len;

  data_buffer_.push_overwrite(uart2can_frame_.frame_head);
  // std::cout << "size=" << data_buffer_.size() << " (cap=" << data_buffer_.capacity() << ")"
  //           << std::endl;
  if (ext == CAN_EXT_FRAME) {
    uart2can_frame_.frame_size = len + 5;
    data_buffer_.push_overwrite(uart2can_frame_.frame_size);
    data_buffer_.push_overwrite(ext);
    for (uint8_t i = 0; i < uart2can_frame_.frame_size; i++) {
      data_buffer_.push_overwrite(buf[i]);
    }
  } else if (ext == CAN_STD_FRAME) {
    uart2can_frame_.frame_size = len + 3;
    data_buffer_.push_overwrite(uart2can_frame_.frame_size);
    data_buffer_.push_overwrite(ext);
    for (uint8_t i = 0; i < uart2can_frame_.frame_size; i++) {
      data_buffer_.push_overwrite(buf[i]);
    }
  }
  data_buffer_.push_overwrite(uart2can_frame_.frame_tail);
  // std::cout << "size=" << data_buffer_.size() << " (cap=" << data_buffer_.capacity() << ")"
  //           << std::endl;

  return true;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
bool UART2CAN::clearMsg(void)
{
  uart2can_frame_.can_id = 0;
  uart2can_frame_.can_dlc = 0;
  uart2can_frame_.can_ext_flag = 0;
  uart2can_frame_.m_nRtr = 0;
  for (int i = 0; i < uart2can_frame_.can_dlc; i++) {
    // uart2can_frame_.data_buffer[i] = 0x00;
  }

  return true;
}

bool UART2CAN::sendMsg(uint8_t * buf, uint8_t rtr, uint8_t ext, uint8_t len)
{
  // printVector(buf, len, "CAN Data to send");
  setMsg(buf, rtr, ext, len);
  return uartTransfer(uart2can_frame_.frame_size);
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
// uint8_t UART2CAN::readMsg(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t buf[])
// {
//   return 0x00;
// }

}  // namespace amp