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

  uart_frame_.frame_head = 0xAA;
  uart_frame_.frame_tail = 0xFF;
}

UART_CAN::~UART_CAN()
{
  serial_close(serial_);
  serial_free(serial_);
}

/*********************************************************************************************************
** Function name:           uartTransfer
** Descriptions:            Performs a uart transfer
*********************************************************************************************************/
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
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
uint8_t UART_CAN::setMode(const uint8_t opMode)
{
  return 0;
}

/*********************************************************************************************************
** Function name:           uart_setCANCTRL_Mode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t UART_CAN::uart_setCANCTRL_Mode(const uint8_t newmode)
{
  return 0;
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
** Function name:           uart_write_canMsg
** Descriptions:            Write message
*********************************************************************************************************/
void UART_CAN::uart_write_canMsg(const uint8_t buffer_sidh_addr)
{
  uint8_t mcp_addr;

  mcp_addr = buffer_sidh_addr;
  uart_setRegisterS(mcp_addr + 5, m_nDta, m_nDlc); /* write data bytes             */

  if (m_nRtr == 1) /* if RTR set bit in byte       */
  {
    m_nDlc |= MCP_RTR_MASK;
  }

  // uart_setRegister((mcp_addr + 4), m_nDlc);  /* write the RTR and DLC        */
  uart_setRegister(0x35, m_nDlc);            /* write the RTR and DLC        */
  uart_write_id(mcp_addr, m_nExtFlg, m_nID); /* write CAN id                 */
}

/*********************************************************************************************************
** Function name:           uart_read_canMsg
** Descriptions:            Read message
*********************************************************************************************************/
void UART_CAN::uart_read_canMsg(const uint8_t buffer_sidh_addr) /* read can msg */
{
  uint8_t ctrl;
  uint8_t mcp_addr;

  mcp_addr = buffer_sidh_addr;

  uart_read_id(mcp_addr, &m_nExtFlg, &m_nID);

  ctrl = uart_readRegister(mcp_addr - 1);
  m_nDlc = uart_readRegister(mcp_addr + 4);

  if (ctrl & 0x08) {
    m_nRtr = 1;
  } else {
    m_nRtr = 0;
  }

  m_nDlc &= MCP_DLC_MASK;
  uart_readRegisterS(mcp_addr + 5, &(m_nDta[0]), m_nDlc);
}

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
uint8_t UART_CAN::initUART2CAN(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
  uint8_t res;

  if (res == uart_OK) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "CAN is initialized");
    return CAN_OK;
  }

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "CAN initializtion is failed");
  return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
uint8_t UART_CAN::setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * buf)
{
  uart_frame_.m_nID = id;
  uart_frame_.m_nRtr = rtr;
  uart_frame_.m_nExtFlg = ext;
  uart_frame_.m_nDlc = FRAME_HEAD_LENGTH + len - 1;
  uart_frame_.m_nDta[0] = m_nDlc;
  if (ext != 0) {
    // m_nDta[1] = (uint16_t)(id >> 16);
  } else {
    uart_frame_.m_nDta[1] = 0x00;
    uart_frame_.m_nDta[2] = (uint16_t)(id >> 16);
    uart_frame_.m_nDta[3] = (uint16_t)(id & 0xFF);
  }

  for (int i = FRAME_HEAD_LENGTH; i < m_nDlc; i++) {
    uart_frame_.m_nDta[i] = buf[i - FRAME_HEAD_LENGTH];
  }

  return uart_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
uint8_t UART_CAN::clearMsg()
{
  uart_frame_.m_nID = 0;
  uart_frame_.m_nDlc = 0;
  uart_frame_.m_nExtFlg = 0;
  uart_frame_.m_nRtr = 0;
  uart_frame_.m_nfilhit = 0;
  for (int i = 0; i < m_nDlc; i++) {
    uart_frame_.m_nDta[i] = 0x00;
  }

  return uart_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t UART_CAN::sendMsg()
{
  uint8_t res, res1, txbuf_n;
  uint16_t uiTimeOut = 0;

  do {
    res = uartTransfer(uart_frame_.m_nDlc, uart_frame_.m_nDta);
    uiTimeOut++;
  } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
void UART_CAN::uartSend(uint32_t canid, uint8_t ext, uint8_t * buf, uint8_t len)
{
  setMsg(canid, 1, ext, len, buf);
}

uint8_t UART_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t * buf)
{
  uint8_t res;

  setMsg(id, 1, ext, len, buf);
  res = sendMsg();

  return res;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
uint8_t UART_CAN::sendMsgBuf(uint32_t id, uint8_t len, uint8_t * buf)
{
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
uint8_t UART_CAN::readMsg()
{
  uint8_t stat, res;

  return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t UART_CAN::readMsgBuf(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t buf[])
{
  return CAN_OK;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t UART_CAN::readMsgBuf(uint32_t * id, uint8_t * len, uint8_t buf[])
{
  return CAN_OK;
}

}  // namespace amp