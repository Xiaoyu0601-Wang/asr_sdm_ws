#include "asr_sdm_hardware/uart_can.hpp"

namespace amp
{

UART_CAN::UART_CAN(const std::string & uart_port, uint32_t uart_baudrate)
{
  uart_port_ = uart_port.c_str();
  serial_ = serial_new();

  /* Open /dev/ttyS3 with baudrate 115200, and defaults of 8N1, no flow control */
  if (serial_open(serial_, uart_port_, uart_baudrate) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "serial_ttyS3_open(): %s", serial_errmsg(serial_));
  }

  uart2can_frame_.frame_head = 0xAA;
  uart2can_frame_.frame_tail = 0xFF;
}

UART_CAN::~UART_CAN()
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
bool UART_CAN::uartTransfer(uint8_t byte_number, unsigned char * tx_buf)
{
  /* Write to the serial port */
  if (serial_write(serial_, tx_buf, byte_number) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "serial_write(): %s\n", serial_errmsg(serial_));
    return false;
  }

  return true;
}

/*********************************************************************************************************
** Function name:           uart_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
uint8_t UART_CAN::uart_configRate(const uint8_t canSpeed, const uint8_t canClock)
{
  return 0;
}

/*********************************************************************************************************
** Function name:           uart_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void UART_CAN::uart_write_id(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
{
}

/*********************************************************************************************************
** Function name:           uart_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
void UART_CAN::uart_read_id(const uint8_t mcp_addr, uint8_t * ext, uint32_t * id)
{
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
uint8_t UART_CAN::setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * buf)
{
  uart2can_frame_.m_nID = id;
  uart2can_frame_.m_nRtr = rtr;
  uart2can_frame_.m_nExtFlg = ext;
  uart2can_frame_.m_nDlc = FRAME_HEAD_LENGTH + len - 1;
  uart2can_frame_.m_nDta[0] = uart2can_frame_.m_nDlc;
  if (ext != 0) {
    // m_nDta[1] = (uint16_t)(id >> 16);
  } else {
    uart2can_frame_.m_nDta[1] = 0x00;
    uart2can_frame_.m_nDta[2] = (uint16_t)(id >> 16);
    uart2can_frame_.m_nDta[3] = (uint16_t)(id & 0xFF);
  }

  for (int i = FRAME_HEAD_LENGTH; i < uart2can_frame_.m_nDlc; i++) {
    uart2can_frame_.m_nDta[i] = buf[i - FRAME_HEAD_LENGTH];
  }

  return 0x00;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
uint8_t UART_CAN::clearMsg()
{
  uart2can_frame_.m_nID = 0;
  uart2can_frame_.m_nDlc = 0;
  uart2can_frame_.m_nExtFlg = 0;
  uart2can_frame_.m_nRtr = 0;
  for (int i = 0; i < uart2can_frame_.m_nDlc; i++) {
    uart2can_frame_.m_nDta[i] = 0x00;
  }

  return 0;
}

uint8_t UART_CAN::sendMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * buf)
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
uint8_t UART_CAN::readMsg(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t buf[])
{
  return 0x00;
}

}  // namespace amp