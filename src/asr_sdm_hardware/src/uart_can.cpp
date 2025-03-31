#include "asr_sdm_hardware/uart_can.hpp"

namespace amp
{

UART_CAN::UART_CAN()
{
  serial_ = serial_new();

  /* Open /dev/ttyS3 with baudrate 115200, and defaults of 8N1, no flow control */
  if (serial_open(serial_, "/dev/ttyS3", 115200) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "serial_ttyS3_open(): %s", serial_errmsg(serial_));
  }

  uart_frame_.frame_head = 0xAA;
  uart_frame_.frame_tail = 0xFF;
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
** Function name:           uart_readRegister
** Descriptions:            Read data register
*********************************************************************************************************/
uint8_t UART_CAN::uart_readRegister(const uint8_t address)
{
  uint8_t rx_buf[3] = {0};
  unsigned char tx_buf[3] = {MCP_READ, address, 0x00};
  spiTransfer(3, tx_buf, rx_buf);
  return rx_buf[2];
}

/*********************************************************************************************************
** Function name:           uart_readRegisterS
** Descriptions:            Reads sucessive data registers
*********************************************************************************************************/
void UART_CAN::uart_readRegisterS(const uint8_t address, uint8_t values[], const uint8_t n)
{
  uint8_t i;

  int buf_size = 3 + n;
  //    unsigned char buf[buf_size] = { 0x00 };
  unsigned char tx_buf[3] = {MCP_READ, address, 0x00};
  unsigned char * rx_buf = (unsigned char *)malloc(buf_size * sizeof(unsigned char));

  spiTransfer(buf_size, tx_buf, rx_buf);

  // uart has auto-increment of address-pointer
  for (i = 0; i < n; ++i) {
    values[i] = rx_buf[i + 2];
  }

  // Free the allocated memory
  free(rx_buf);
}

/*********************************************************************************************************
** Function name:           uart_setRegister
** Descriptions:            Sets data register
*********************************************************************************************************/
void UART_CAN::uart_setRegister(const uint8_t address, const uint8_t value)
{
  unsigned char tx_buf[3] = {MCP_WRITE, address, value};
  unsigned char rx_buf[3] = {0};

  spiTransfer(3, tx_buf, rx_buf);
}

/*********************************************************************************************************
** Function name:           uart_setRegisterS
** Descriptions:            Sets sucessive data registers
*********************************************************************************************************/
void UART_CAN::uart_setRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n)
{
  int buf_size = 3 + n;
  unsigned char tx_buf[4] = {MCP_WRITE, address, 0x00, 0x00};
  // unsigned char * tx_buf = (unsigned char *)malloc(buf_size * sizeof(unsigned char));
  unsigned char rx_buf[4] = {0};

  for (uint8_t i = 0; i < n; ++i) {
    tx_buf[2] = values[i];
    spiTransfer(4, tx_buf, rx_buf);
    tx_buf[1] += 1;
  }

  // tx_buf[buf_size - 1] = 0x00;

  // spiTransfer(buf_size, tx_buf, rx_buf);

  // Free the allocated memory
  // free(tx_buf);
}

/*********************************************************************************************************
** Function name:           uart_modifyRegister
** Descriptions:            Sets specific bits of a register
*********************************************************************************************************/
void UART_CAN::uart_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
  unsigned char tx_buf[5] = {MCP_BITMOD, address, mask, data, 0x00};
  unsigned char rx_buf[5] = {0};

  spiTransfer(5, tx_buf, rx_buf);
}

/*********************************************************************************************************
** Function name:           uart_readStatus
** Descriptions:            Reads status register
*********************************************************************************************************/
uint8_t UART_CAN::uart_readStatus(void)
{
  unsigned char buf[2] = {MCP_READ_STATUS, 0x00};
  spiTransfer(2, buf, buf);

  return buf[1];
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
uint8_t UART_CAN::setMode(const uint8_t opMode)
{
  this->mcpMode = opMode;
  // digitalWrite(this->gpio_can_cs, LOW);
  return uart_setCANCTRL_Mode(this->mcpMode);
}

/*********************************************************************************************************
** Function name:           uart_setCANCTRL_Mode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t UART_CAN::uart_setCANCTRL_Mode(const uint8_t newmode)
{
  // uart_modifyRegister(UART_CANCTRL, MODE_MASK, newmode);
  uart_setRegister(UART_CANCTRL, newmode);
  rclcpp::sleep_for(std::chrono::milliseconds(1));
  uint8_t mode_status = uart_readRegister(UART_CANCTRL);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "mode command: %#04x", newmode);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "mode_status: %#04x", mode_status);
  if (mode_status == newmode) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "uart Configuration Mode Successful!");
    return uart_OK;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "uart Configuration Mode Failure...");
    return uart_FAIL;
  }
}

/*********************************************************************************************************
** Function name:           uart_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
uint8_t UART_CAN::uart_configRate(const uint8_t canSpeed, const uint8_t canClock)
{
}

/*********************************************************************************************************
** Function name:           uart_initCANBuffers
** Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/
void UART_CAN::uart_initCANBuffers(void)
{
  uint8_t i, a1, a2, a3;

  uint8_t std = 0;
  uint8_t ext = 1;
  uint32_t ulMask = 0x00, ulFilt = 0x00;

  uart_write_mf(MCP_RXM0SIDH, ext, ulMask); /*Set both masks to 0           */
  uart_write_mf(MCP_RXM1SIDH, ext, ulMask); /*Mask register ignores ext bit */

  /* Set all filters to 0         */
  uart_write_mf(MCP_RXF0SIDH, ext, ulFilt); /* RXB0: extended               */
  uart_write_mf(MCP_RXF1SIDH, std, ulFilt); /* RXB1: standard               */
  uart_write_mf(MCP_RXF2SIDH, ext, ulFilt); /* RXB2: extended               */
  uart_write_mf(MCP_RXF3SIDH, std, ulFilt); /* RXB3: standard               */
  uart_write_mf(MCP_RXF4SIDH, ext, ulFilt);
  uart_write_mf(MCP_RXF5SIDH, std, ulFilt);

  /* Clear, deactivate the three  */
  /* transmit buffers             */
  /* TXBnCTRL -> TXBnD7           */
  a1 = MCP_TXB0CTRL;
  a2 = MCP_TXB1CTRL;
  a3 = MCP_TXB2CTRL;
  for (i = 0; i < 14; i++) /* in-buffer loop               */
  {
    uart_setRegister(a1, 0);
    uart_setRegister(a2, 0);
    uart_setRegister(a3, 0);
    a1++;
    a2++;
    a3++;
  }
  uart_setRegister(MCP_RXB0CTRL, 0);
  uart_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           inituart
** Descriptions:            Initialize the controller
*********************************************************************************************************/
uint8_t UART_CAN::inituart(const uint8_t canIDMode, const uint8_t canSpeed, const uint8_t canClock)
{
  // Reset uart
  resetuart();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  mcpMode = MCP_NORMAL | CLKOUT_ENABLE;
  uint8_t dummy = uart_readRegister(UART_CANSTAT);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "UART_CANSTAT: %#04x", dummy);

  // if (uart_setCANCTRL_Mode(MODE_CONFIG | CLKOUT_ENABLE) > 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering uart Configuration Mode
  //   Failure..."); return uart_FAIL;
  // }

  // Set Baudrate
  if (uart_configRate(canSpeed, canClock)) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting CAN Baudrate Failure...");
    return uart_FAIL;
  }
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting CAN Baudrate Successful!");

  uart_setRegister(0x31, 0xFF);
  uart_setRegister(0x32, 0xE0);
  // uart_setRegister(0x33, 0xFF);
  // uart_setRegister(0x34, 0xFF);
  // uart_setRegister(0x35, 0x40 | 0x08);

  /* init canbuffers              */
  // uart_initCANBuffers();

  // #Set RX
  uart_setRegister(MCP_RXB0SIDH, 0x00);
  uart_setRegister(0x62, 0x00);
  // uart_setRegister(0x63, 0x00);
  // uart_setRegister(0x64, 0x00);
  uart_setRegister(MCP_RXB0CTRL, 0x20);
  uart_setRegister(0x65, 0x08);  // DLC_8

  uart_setRegister(MCP_RXF0SIDH, 0xFF);
  uart_setRegister(MCP_RXF0SIDL, 0xE0);
  uart_setRegister(MCP_RXM0SIDH, 0xFF);
  uart_setRegister(MCP_RXM0SIDL, 0xE0);

  /* interrupt mode               */
  uart_setRegister(UART_CANINTF, 0x00);
  uart_setRegister(UART_CANINTE, 0x01);  // MCP_RX0IF);  // | MCP_RX1IF);

  if (uart_setCANCTRL_Mode(MCP_NORMAL | CLKOUT_ENABLE)) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Returning to Previous Mode Failure...");
    return uart_FAIL;
  }
  dummy = uart_readRegister(UART_CANSTAT);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "UART_CANSTAT: %#04x", dummy);

  return uart_OK;
}

/*********************************************************************************************************
** Function name:           uart_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void UART_CAN::uart_write_id(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
{
  uint16_t canid;
  uint8_t tbufdata[4];

  canid = (uint16_t)(id & 0x0FFFF);

  // if (ext == 1) {
  //   tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
  //   tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
  //   canid = (uint16_t)(id >> 16);
  //   tbufdata[MCP_SIDL] = (uint8_t)(canid & 0x03);
  //   tbufdata[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
  //   tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
  //   tbufdata[MCP_SIDH] = (uint8_t)(canid >> 5);
  // } else {
  //   tbufdata[MCP_SIDH] = (uint8_t)(canid >> 3);
  //   tbufdata[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
  //   tbufdata[MCP_EID0] = 0;
  //   tbufdata[MCP_EID8] = 0;
  // }

  // uart_setRegisterS(mcp_addr, tbufdata, 4);

  uart_setRegister(0x31, (canid >> 3) & 0XFF);
  uart_setRegister(0x32, (canid & 0x07) << 5);
  uart_setRegister(0x33, 0);
  uart_setRegister(0x34, 0);
  // uart_setRegister(TXB0DLC, len);
}

/*********************************************************************************************************
** Function name:           uart_write_mf
** Descriptions:            Write Masks and Filters
*********************************************************************************************************/
void UART_CAN::uart_write_mf(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
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
** Function name:           uart_getNextFreeTXBuf
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t UART_CAN::uart_getNextFreeTXBuf(uint8_t * txbuf_address) /* get Next free txbuf */
{
  uint8_t i, ctrlval;
  uint8_t ctrlregs[MCP_N_TXBUFFERS] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};

  *txbuf_address = 0x00;

  /* check all 3 TX-Buffers       */
  for (i = 0; i < MCP_N_TXBUFFERS; i++) {
    ctrlval = uart_readRegister(ctrlregs[i]);
    if ((ctrlval & MCP_TXB_TXREQ_M) == 0) {
      *txbuf_address = ctrlregs[i] + 1; /* return SIDH-address of Buffer*/
      return uart_OK;                   /* ! function exit */
    }
  }
  return MCP_ALLTXBUSY;
}

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
uint8_t UART_CAN::initUART2CAN(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
  uint8_t res;

  res = inituart(idmodeset, speedset, clockset);
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

  if (uiTimeOut == TIMEOUTVALUE) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Get uart Tx Buff Time Out!");
    return CAN_GETTXBFTIMEOUT; /* get tx buff time out         */
  }
  uiTimeOut = 0;
  uart_write_canMsg(txbuf_n);
  uart_modifyRegister(txbuf_n - 1, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);

  do {
    uiTimeOut++;
    res1 = uart_readRegister(txbuf_n - 1); /* read send buff ctrl reg  */
    res1 = res1 & 0x08;
  } while (res1 && (uiTimeOut < TIMEOUTVALUE));

  if (uiTimeOut == TIMEOUTVALUE) /* send msg timeout             */
  {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "CAN Send Msg Time Out!");
    return CAN_SENDMSGTIMEOUT;
  }

  return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
void UART_CAN::uartSend(uint32_t canid, uint8_t ext, uint8_t * buf, uint8_t len)
{
  setMsg(canid, 1, ext, len, buf);

  // RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP_CANSTAT: %#04x", dummy);
  /* Write to the serial port */
  if (serial_write(serial_, s_buf, sizeof(s_buf)) < 0) {
    fprintf(stderr, "serial_write(): %s\n", serial_errmsg(serial_));
  }
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
  uint8_t ext = 0, rtr = 0;
  uint8_t res;

  if ((id & 0x80000000) == 0x80000000) {
    ext = 1;
  }

  if ((id & 0x40000000) == 0x40000000) {
    rtr = 1;
  }

  setMsg(id, rtr, ext, len, buf);
  res = sendMsg();

  return res;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
uint8_t UART_CAN::readMsg()
{
  uint8_t stat, res;

  stat = uart_readStatus();

  if (stat & MCP_STAT_RX0IF) /* Msg in Buffer 0              */
  {
    uart_read_canMsg(MCP_RXBUF_0);
    uart_modifyRegister(UART_CANINTF, MCP_RX0IF, 0);
    res = CAN_OK;
  } else if (stat & MCP_STAT_RX1IF) /* Msg in Buffer 1              */
  {
    uart_read_canMsg(MCP_RXBUF_1);
    uart_modifyRegister(UART_CANINTF, MCP_RX1IF, 0);
    res = CAN_OK;
  } else {
    res = CAN_NOMSG;
  }

  return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t UART_CAN::readMsgBuf(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t buf[])
{
  if (readMsg() == CAN_NOMSG) {
    return CAN_NOMSG;
  }

  *id = m_nID;
  *len = m_nDlc;
  *ext = m_nExtFlg;
  for (int i = 0; i < m_nDlc; i++) {
    buf[i] = m_nDta[i];
  }

  return CAN_OK;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t UART_CAN::readMsgBuf(uint32_t * id, uint8_t * len, uint8_t buf[])
{
  if (readMsg() == CAN_NOMSG) {
    return CAN_NOMSG;
  }

  if (m_nExtFlg) {
    m_nID |= 0x80000000;
  }

  if (m_nRtr) {
    m_nID |= 0x40000000;
  }

  *id = m_nID;
  *len = m_nDlc;

  for (int i = 0; i < m_nDlc; i++) {
    buf[i] = m_nDta[i];
  }

  return CAN_OK;
}

}  // namespace amp