#include "asr_sdm_hardware/uart2can.hpp"

#include <vector>

namespace amp
{

UART2CAN::UART2CAN(
  const std::string & uart_port, uint32_t uart_baudrate, uint32_t can_id, uint8_t can_frame_length,
  uint8_t uart_frame_head, uint8_t uart_frame_tail)
: data_buffer_(MAX_FRAME_DATA_LENGTH)  // initialize with desired capacity
{
  uart_port_ = uart_port.c_str();
  serial_ = serial_new();

  /* Open /dev/ttyS3 with baudrate 115200, and defaults of 8N1, no flow control */
  if (serial_open(serial_, uart_port_, uart_baudrate) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "serial_ttyS3_open(): %s", serial_errmsg(serial_));
  }

  uart2can_frame_.can_id = can_id;
  uart2can_frame_.can_dlc = can_frame_length;
  uart2can_frame_.frame_head = uart_frame_head;
  uart2can_frame_.frame_tail = uart_frame_tail;
}

UART2CAN::~UART2CAN()
{
  serial_close(serial_);
  serial_free(serial_);
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
  std::vector<uint8_t> tx_buf(byte_number);
  uint8_t popped_val;
  while (data_buffer_.pop(popped_val)) {
    if (popped_val == uart2can_frame_.frame_head) {
      tx_buf[0] = uart2can_frame_.frame_head;
      continue;
    }
  }
  for (uint8_t i = 1; i < byte_number - 1; i++) {
    data_buffer_.pop(popped_val);
    tx_buf[i] = popped_val;
  }
  data_buffer_.pop(popped_val);
  if (popped_val == uart2can_frame_.frame_tail) {
    tx_buf[byte_number - 1] = uart2can_frame_.frame_tail;
  } else {
    return false;
  }

  /* Write to the serial port */
  if (serial_write(serial_, tx_buf.data(), byte_number) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "serial_write(): %s\n", serial_errmsg(serial_));
    return false;
  }

  return true;
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
bool UART2CAN::setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * buf)
{
  uart2can_frame_.can_id = id;
  uart2can_frame_.m_nRtr = rtr;
  uart2can_frame_.can_ext_flag = ext;
  data_buffer_.push_overwrite(uart2can_frame_.frame_head);
  data_buffer_.push_overwrite(len);
  if (ext != 0) {
    data_buffer_.push_overwrite(0x80);
  } else {
    data_buffer_.push_overwrite(0x00);
  }
  data_buffer_.push_overwrite((uint8_t)(id >> 24));
  data_buffer_.push_overwrite((uint8_t)(id >> 16));
  data_buffer_.push_overwrite((uint8_t)(id >> 8));
  data_buffer_.push_overwrite((uint8_t)(id & 0xFF));

  for (int i = 0; i < uart2can_frame_.can_dlc; i++) {
    data_buffer_.push_overwrite(buf[i]);
  }
  data_buffer_.push_overwrite(uart2can_frame_.frame_tail);

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

uint8_t UART2CAN::sendMsg(uint32_t id, uint8_t rtr, bool ext, uint8_t len, uint8_t * buf)
{
  uint8_t res;

  setMsg(id, rtr, ext, len, buf);
  // res = sendMsg();

  return res;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t UART2CAN::readMsg(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t buf[])
{
  return 0x00;
}

}  // namespace amp