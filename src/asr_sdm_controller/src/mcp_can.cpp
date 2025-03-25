#include "asr_sdm_controller/mcp_can.hpp"

namespace amp
{
/*********************************************************************************************************
** Function name:           spiTransfer
** Descriptions:            Performs a spi transfer on Raspberry Pi (using wiringPi)
*********************************************************************************************************/
bool MCP_CAN::spiTransfer(uint8_t byte_number, unsigned char * tx_buf, unsigned char * rx_buf)
{
  if (spi_transfer(spi_, tx_buf, rx_buf, byte_number) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "failed");
    fprintf(stderr, "spi_transfer(): %s\n", spi_errmsg(spi_));
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("hardware"), "cmd: %#04x, %#04x, %#04x", tx_buf[0], tx_buf[1], tx_buf[2]);
  RCLCPP_INFO(
    rclcpp::get_logger("hardware"), "%#04x, %#04x, %#04x, %#04x", rx_buf[0], rx_buf[1], rx_buf[2],
    rx_buf[3]);

  // usleep(10);
  return true;
}

/*********************************************************************************************************
** Function name:           setupInterruptGpio
** Descriptions:            Setups interrupt GPIO pin as input on Raspberry Pi (using wiringPi)
*********************************************************************************************************/
// bool MCP_CAN::setupInterruptGpio()
// {
// 	wiringPiSetup();
//     int result = wiringPiSetupGpio();
//     if (!result)
//     {
//         printf("Gpio started\n");
//     }
//     else
//     {
//         printf("Gpio startup fail\n");
//         return false;
//     }

//     pinMode(this->gpio_can_interrupt, INPUT);
// //    wiringPiSetupGpio();
//     pinMode(this->gpio_can_cs, OUTPUT);//(8, OUTPUT);//
//     digitalWrite(this->gpio_can_cs, LOW);//(8, LOW);//HIGH

//     struct timespec req;
//     req.tv_sec = 0;        // seconds
//     req.tv_nsec = 500000L; // nanoseconds
//     nanosleep(&req, nullptr);
// //    nanosleep((const struct timespec[]){ { 0, 500000L } }, NULL);

//     return true;
// }

/*********************************************************************************************************
** Function name:           setupSpi
** Descriptions:            Setups spi communication on Raspberry Pi (using wiringPi)
*********************************************************************************************************/
// bool MCP_CAN::setupSpi()
// {
// //    int result_spi = wiringPiSPISetup(spi_channel, spi_baudrate);
// //    printf("Started SPI : %d\n", result_spi);
// //    if (result_spi < 0)
// //    {
// //        return false;
// //    }
// //	digitalWrite(gpio_can_cs, LOW);
// 	if (wiringPiSPISetup(this->spi_channel, this->spi_baudrate) < 0)
// 	{
// 		fprintf(stderr, "Can't open the SPI bus: %s\n", strerror(errno));
// 		exit(EXIT_FAILURE);
// 	}

//     struct timespec req;
//     req.tv_sec = 0;        // seconds
//     req.tv_nsec = 500000L; // nanoseconds
//     nanosleep(&req, nullptr);
// //    nanosleep((const struct timespec[]){ { 0, 500000L } }, NULL);
// //    digitalWrite(gpio_can_cs, LOW);

//     return true;
// }

/*********************************************************************************************************
** Function name:           canReadData
** Descriptions:            Checks GPIO interrupt pin to see if data is available (using wiringPi)
*********************************************************************************************************/
// bool MCP_CAN::canReadData()
// {
//     return !digitalRead(this->gpio_can_interrupt);
// }

/*********************************************************************************************************
** Function name:           resetMCP2515
** Descriptions:            Performs a software reset
*********************************************************************************************************/
bool MCP_CAN::resetMCP2515(void)
{
  unsigned char tx_buf[2] = {MCP_RESET, 0x00};
  unsigned char rx_buf[2] = {0};

  if (!spiTransfer(2, tx_buf, rx_buf)) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP2515 Reset Failure...");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP2515 Reset  Successful!");
  return true;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            Read data register
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_readRegister(const uint8_t address)
{
  uint8_t rx_buf[3] = {0};
  unsigned char tx_buf[3] = {MCP_READ, address, 0x00};
  spiTransfer(3, tx_buf, rx_buf);
  return rx_buf[2];
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            Reads sucessive data registers
*********************************************************************************************************/
void MCP_CAN::mcp2515_readRegisterS(const uint8_t address, uint8_t values[], const uint8_t n)
{
  uint8_t i;

  int buf_size = 3 + n;
  //    unsigned char buf[buf_size] = { 0x00 };
  unsigned char tx_buf[3] = {MCP_READ, address, 0x00};
  unsigned char * rx_buf = (unsigned char *)malloc(buf_size * sizeof(unsigned char));

  spiTransfer(buf_size, tx_buf, rx_buf);

  // mcp2515 has auto-increment of address-pointer
  for (i = 0; i < n; ++i) {
    values[i] = rx_buf[i + 2];
  }

  // Free the allocated memory
  free(rx_buf);
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            Sets data register
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegister(const uint8_t address, const uint8_t value)
{
  unsigned char tx_buf[3] = {MCP_WRITE, address, value};
  unsigned char rx_buf[3] = {0};

  spiTransfer(3, tx_buf, rx_buf);
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            Sets sucessive data registers
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n)
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
** Function name:           mcp2515_modifyRegister
** Descriptions:            Sets specific bits of a register
*********************************************************************************************************/
void MCP_CAN::mcp2515_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
  unsigned char tx_buf[5] = {MCP_BITMOD, address, mask, data, 0x00};
  unsigned char rx_buf[5] = {0};

  spiTransfer(5, tx_buf, rx_buf);
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            Reads status register
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_readStatus(void)
{
  unsigned char buf[2] = {MCP_READ_STATUS, 0x00};
  spiTransfer(2, buf, buf);

  return buf[1];
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
uint8_t MCP_CAN::setMode(const uint8_t opMode)
{
  this->mcpMode = opMode;
  // digitalWrite(this->gpio_can_cs, LOW);
  return mcp2515_setCANCTRL_Mode(this->mcpMode);
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_setCANCTRL_Mode(const uint8_t newmode)
{
  // mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);
  mcp2515_setRegister(MCP_CANCTRL, newmode);
  rclcpp::sleep_for(std::chrono::milliseconds(1));
  uint8_t mode_status = mcp2515_readRegister(MCP_CANCTRL);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "mode command: %#04x", newmode);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "mode_status: %#04x", mode_status);
  if (mode_status == newmode) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP2515 Configuration Mode Successful!");
    return MCP2515_OK;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP2515 Configuration Mode Failure...");
    return MCP2515_FAIL;
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_configRate(const uint8_t canSpeed, const uint8_t canClock)
{
  uint8_t set, cfg1, cfg2, cfg3;

  set = 1;
  switch (canClock) {
    case (MCP_8MHZ):
      switch (canSpeed) {
        case (CAN_5KBPS):  //   5KBPS
          cfg1 = MCP_8MHz_5kBPS_CFG1;
          cfg2 = MCP_8MHz_5kBPS_CFG2;
          cfg3 = MCP_8MHz_5kBPS_CFG3;
          break;

        case (CAN_10KBPS):  //  10KBPS
          cfg1 = MCP_8MHz_10kBPS_CFG1;
          cfg2 = MCP_8MHz_10kBPS_CFG2;
          cfg3 = MCP_8MHz_10kBPS_CFG3;
          break;

        case (CAN_20KBPS):  //  20KBPS
          cfg1 = MCP_8MHz_20kBPS_CFG1;
          cfg2 = MCP_8MHz_20kBPS_CFG2;
          cfg3 = MCP_8MHz_20kBPS_CFG3;
          break;

        case (CAN_31K25BPS):  //  31.25KBPS
          cfg1 = MCP_8MHz_31k25BPS_CFG1;
          cfg2 = MCP_8MHz_31k25BPS_CFG2;
          cfg3 = MCP_8MHz_31k25BPS_CFG3;
          break;

        case (CAN_33K3BPS):  //  33.33KBPS
          cfg1 = MCP_8MHz_33k3BPS_CFG1;
          cfg2 = MCP_8MHz_33k3BPS_CFG2;
          cfg3 = MCP_8MHz_33k3BPS_CFG3;
          break;

        case (CAN_40KBPS):  //  40Kbps
          cfg1 = MCP_8MHz_40kBPS_CFG1;
          cfg2 = MCP_8MHz_40kBPS_CFG2;
          cfg3 = MCP_8MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS):  //  50Kbps
          cfg1 = MCP_8MHz_50kBPS_CFG1;
          cfg2 = MCP_8MHz_50kBPS_CFG2;
          cfg3 = MCP_8MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS):  //  80Kbps
          cfg1 = MCP_8MHz_80kBPS_CFG1;
          cfg2 = MCP_8MHz_80kBPS_CFG2;
          cfg3 = MCP_8MHz_80kBPS_CFG3;
          break;

        case (CAN_100KBPS):  // 100Kbps
          cfg1 = MCP_8MHz_100kBPS_CFG1;
          cfg2 = MCP_8MHz_100kBPS_CFG2;
          cfg3 = MCP_8MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS):  // 125Kbps
          cfg1 = MCP_8MHz_125kBPS_CFG1;
          cfg2 = MCP_8MHz_125kBPS_CFG2;
          cfg3 = MCP_8MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS):  // 200Kbps
          cfg1 = MCP_8MHz_200kBPS_CFG1;
          cfg2 = MCP_8MHz_200kBPS_CFG2;
          cfg3 = MCP_8MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS):  // 250Kbps
          cfg1 = MCP_8MHz_250kBPS_CFG1;
          cfg2 = MCP_8MHz_250kBPS_CFG2;
          cfg3 = MCP_8MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):  // 500Kbps
          cfg1 = MCP_8MHz_500kBPS_CFG1;
          cfg2 = MCP_8MHz_500kBPS_CFG2;
          cfg3 = MCP_8MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS):  //   1Mbps
          cfg1 = MCP_8MHz_1000kBPS_CFG1;
          cfg2 = MCP_8MHz_1000kBPS_CFG2;
          cfg3 = MCP_8MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          return MCP2515_FAIL;

          break;
      }
      break;

    case (MCP_12MHZ):
      switch (canSpeed) {
        case (CAN_500KBPS):  // 500Kbps
          cfg1 = MCP_8MHz_500kBPS_CFG1;
          cfg2 = MCP_8MHz_500kBPS_CFG2;
          cfg3 = MCP_8MHz_500kBPS_CFG3;
          break;

        default:
          set = 0;
          return MCP2515_FAIL;

          break;
      }
      break;

    case (MCP_16MHZ):
      switch (canSpeed) {
        case (CAN_5KBPS):  //   5Kbps
          cfg1 = MCP_16MHz_5kBPS_CFG1;
          cfg2 = MCP_16MHz_5kBPS_CFG2;
          cfg3 = MCP_16MHz_5kBPS_CFG3;
          break;

        case (CAN_10KBPS):  //  10Kbps
          cfg1 = MCP_16MHz_10kBPS_CFG1;
          cfg2 = MCP_16MHz_10kBPS_CFG2;
          cfg3 = MCP_16MHz_10kBPS_CFG3;
          break;

        case (CAN_20KBPS):  //  20Kbps
          cfg1 = MCP_16MHz_20kBPS_CFG1;
          cfg2 = MCP_16MHz_20kBPS_CFG2;
          cfg3 = MCP_16MHz_20kBPS_CFG3;
          break;

        case (CAN_33K3BPS):  //  20Kbps
          cfg1 = MCP_16MHz_33k3BPS_CFG1;
          cfg2 = MCP_16MHz_33k3BPS_CFG2;
          cfg3 = MCP_16MHz_33k3BPS_CFG3;
          break;

        case (CAN_40KBPS):  //  40Kbps
          cfg1 = MCP_16MHz_40kBPS_CFG1;
          cfg2 = MCP_16MHz_40kBPS_CFG2;
          cfg3 = MCP_16MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS):  //  50Kbps
          cfg2 = MCP_16MHz_50kBPS_CFG2;
          cfg3 = MCP_16MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS):  //  80Kbps
          cfg1 = MCP_16MHz_80kBPS_CFG1;
          cfg2 = MCP_16MHz_80kBPS_CFG2;
          cfg3 = MCP_16MHz_80kBPS_CFG3;
          break;

        case (CAN_100KBPS):  // 100Kbps
          cfg1 = MCP_16MHz_100kBPS_CFG1;
          cfg2 = MCP_16MHz_100kBPS_CFG2;
          cfg3 = MCP_16MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS):  // 125Kbps
          cfg1 = MCP_16MHz_125kBPS_CFG1;
          cfg2 = MCP_16MHz_125kBPS_CFG2;
          cfg3 = MCP_16MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS):  // 200Kbps
          cfg1 = MCP_16MHz_200kBPS_CFG1;
          cfg2 = MCP_16MHz_200kBPS_CFG2;
          cfg3 = MCP_16MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS):  // 250Kbps
          cfg1 = MCP_16MHz_250kBPS_CFG1;
          cfg2 = MCP_16MHz_250kBPS_CFG2;
          cfg3 = MCP_16MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):  // 500Kbps
          cfg1 = MCP_16MHz_500kBPS_CFG1;
          cfg2 = MCP_16MHz_500kBPS_CFG2;
          cfg3 = MCP_16MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS):  //   1Mbps
          cfg1 = MCP_16MHz_1000kBPS_CFG1;
          cfg2 = MCP_16MHz_1000kBPS_CFG2;
          cfg3 = MCP_16MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          return MCP2515_FAIL;

          break;
      }
      break;

    case (MCP_20MHZ):
      switch (canSpeed) {
        case (CAN_40KBPS):  //  40Kbps
          cfg1 = MCP_20MHz_40kBPS_CFG1;
          cfg2 = MCP_20MHz_40kBPS_CFG2;
          cfg3 = MCP_20MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS):  //  50Kbps
          cfg1 = MCP_20MHz_50kBPS_CFG1;
          cfg2 = MCP_20MHz_50kBPS_CFG2;
          cfg3 = MCP_20MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS):  //  80Kbps
          cfg1 = MCP_20MHz_80kBPS_CFG1;
          cfg2 = MCP_20MHz_80kBPS_CFG2;
          cfg3 = MCP_20MHz_80kBPS_CFG3;
          break;

        case (CAN_100KBPS):  // 100Kbps
          cfg1 = MCP_20MHz_100kBPS_CFG1;
          cfg2 = MCP_20MHz_100kBPS_CFG2;
          cfg3 = MCP_20MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS):  // 125Kbps
          cfg1 = MCP_20MHz_125kBPS_CFG1;
          cfg2 = MCP_20MHz_125kBPS_CFG2;
          cfg3 = MCP_20MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS):  // 200Kbps
          cfg1 = MCP_20MHz_200kBPS_CFG1;
          cfg2 = MCP_20MHz_200kBPS_CFG2;
          cfg3 = MCP_20MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS):  // 250Kbps
          cfg1 = MCP_20MHz_250kBPS_CFG1;
          cfg2 = MCP_20MHz_250kBPS_CFG2;
          cfg3 = MCP_20MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):  // 500Kbps
          cfg1 = MCP_20MHz_500kBPS_CFG1;
          cfg2 = MCP_20MHz_500kBPS_CFG2;
          cfg3 = MCP_20MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS):  //   1Mbps
          cfg1 = MCP_20MHz_1000kBPS_CFG1;
          cfg2 = MCP_20MHz_1000kBPS_CFG2;
          cfg3 = MCP_20MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          return MCP2515_FAIL;

          break;
      }
      break;

    default:
      set = 0;
      return MCP2515_FAIL;

      break;
  }

  if (set) {
    // mcp2515_setRegister(MCP_CNF1, cfg1);
    // mcp2515_setRegister(MCP_CNF2, cfg2);
    // mcp2515_setRegister(MCP_CNF3, cfg3);
    // mcp2515_setRegister(MCP_CNF1, 0x00);
    // mcp2515_setRegister(MCP_CNF2, 0xd1);
    // mcp2515_setRegister(MCP_CNF3, 0x81);

    // mcp2515_setRegister(MCP_CNF1, 0x00);
    // mcp2515_setRegister(MCP_CNF2, 0x90);
    // mcp2515_setRegister(MCP_CNF3, 0x82);

    // mcp2515_setRegister(MCP_CNF1, 0x00);
    // mcp2515_setRegister(MCP_CNF2, 0xB1);
    // mcp2515_setRegister(MCP_CNF3, 0x85);

    mcp2515_setRegister(MCP_CNF1, 0x00);
    mcp2515_setRegister(MCP_CNF2, 0x90);
    mcp2515_setRegister(MCP_CNF3, 0x02);

    // mcp2515_setRegister(MCP_CNF1, 0x1F);
    // mcp2515_setRegister(MCP_CNF2, 0xBF);
    // mcp2515_setRegister(MCP_CNF3, 0x87);
    return MCP2515_OK;
  }

  return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/
void MCP_CAN::mcp2515_initCANBuffers(void)
{
  uint8_t i, a1, a2, a3;

  uint8_t std = 0;
  uint8_t ext = 1;
  uint32_t ulMask = 0x00, ulFilt = 0x00;

  mcp2515_write_mf(MCP_RXM0SIDH, ext, ulMask); /*Set both masks to 0           */
  mcp2515_write_mf(MCP_RXM1SIDH, ext, ulMask); /*Mask register ignores ext bit */

  /* Set all filters to 0         */
  mcp2515_write_mf(MCP_RXF0SIDH, ext, ulFilt); /* RXB0: extended               */
  mcp2515_write_mf(MCP_RXF1SIDH, std, ulFilt); /* RXB1: standard               */
  mcp2515_write_mf(MCP_RXF2SIDH, ext, ulFilt); /* RXB2: extended               */
  mcp2515_write_mf(MCP_RXF3SIDH, std, ulFilt); /* RXB3: standard               */
  mcp2515_write_mf(MCP_RXF4SIDH, ext, ulFilt);
  mcp2515_write_mf(MCP_RXF5SIDH, std, ulFilt);

  /* Clear, deactivate the three  */
  /* transmit buffers             */
  /* TXBnCTRL -> TXBnD7           */
  a1 = MCP_TXB0CTRL;
  a2 = MCP_TXB1CTRL;
  a3 = MCP_TXB2CTRL;
  for (i = 0; i < 14; i++) /* in-buffer loop               */
  {
    mcp2515_setRegister(a1, 0);
    mcp2515_setRegister(a2, 0);
    mcp2515_setRegister(a3, 0);
    a1++;
    a2++;
    a3++;
  }
  mcp2515_setRegister(MCP_RXB0CTRL, 0);
  mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           initMCP2515
** Descriptions:            Initialize the controller
*********************************************************************************************************/
uint8_t MCP_CAN::initMCP2515(
  const uint8_t canIDMode, const uint8_t canSpeed, const uint8_t canClock)
{
  // Reset MCP2515
  resetMCP2515();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  mcpMode = MCP_NORMAL | CLKOUT_ENABLE;
  uint8_t dummy = mcp2515_readRegister(MCP_CANSTAT);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP_CANSTAT: %#04x", dummy);

  // if (mcp2515_setCANCTRL_Mode(MODE_CONFIG | CLKOUT_ENABLE) > 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering MCP2515 Configuration Mode
  //   Failure..."); return MCP2515_FAIL;
  // }

  // Set Baudrate
  if (mcp2515_configRate(canSpeed, canClock)) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting CAN Baudrate Failure...");
    return MCP2515_FAIL;
  }
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting CAN Baudrate Successful!");

  mcp2515_setRegister(0x31, 0xFF);
  mcp2515_setRegister(0x32, 0xE0);
  // mcp2515_setRegister(0x33, 0xFF);
  // mcp2515_setRegister(0x34, 0xFF);
  // mcp2515_setRegister(0x35, 0x40 | 0x08);

  /* init canbuffers              */
  // mcp2515_initCANBuffers();

  // #Set RX
  mcp2515_setRegister(MCP_RXB0SIDH, 0x00);
  mcp2515_setRegister(0x62, 0x00);
  // mcp2515_setRegister(0x63, 0x00);
  // mcp2515_setRegister(0x64, 0x00);
  mcp2515_setRegister(MCP_RXB0CTRL, 0x20);
  mcp2515_setRegister(0x65, 0x08);  // DLC_8

  mcp2515_setRegister(MCP_RXF0SIDH, 0xFF);
  mcp2515_setRegister(MCP_RXF0SIDL, 0xE0);
  mcp2515_setRegister(MCP_RXM0SIDH, 0xFF);
  mcp2515_setRegister(MCP_RXM0SIDL, 0xE0);

  /* interrupt mode               */
  mcp2515_setRegister(MCP_CANINTF, 0x00);
  mcp2515_setRegister(MCP_CANINTE, 0x01);  // MCP_RX0IF);  // | MCP_RX1IF);

  // switch (canIDMode) {
  //   case (MCP_ANY):
  //     mcp2515_modifyRegister(
  //       MCP_RXB0CTRL, MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK, MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
  //     mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_ANY);
  //     break;

  //     /*          The followingn two functions of the MCP2515 do not work, there is a bug in the
  //      * silicon. case (MCP_STD): mcp2515_modifyRegister(MCP_RXB0CTRL, MCP_RXB_RX_MASK |
  //      * MCP_RXB_BUKT_MASK, MCP_RXB_RX_STD | MCP_RXB_BUKT_MASK );
  //      *          mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
  //      *          MCP_RXB_RX_STD);
  //      *          break;
  //      *          case (MCP_EXT):
  //      *          mcp2515_modifyRegister(MCP_RXB0CTRL,
  //      *          MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
  //      *          MCP_RXB_RX_EXT | MCP_RXB_BUKT_MASK );
  //      *          mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
  //      *          MCP_RXB_RX_EXT);
  //      *          break;
  //      */
  //   case (MCP_STDEXT):
  //     mcp2515_modifyRegister(
  //       MCP_RXB0CTRL, MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK, MCP_RXB_RX_STDEXT |
  //       MCP_RXB_BUKT_MASK);
  //     mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_STDEXT);
  //     break;

  //   default:
  //     RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting ID Mode Failure...");
  //     return MCP2515_FAIL;

  //     break;
  // }

  if (mcp2515_setCANCTRL_Mode(MCP_NORMAL | CLKOUT_ENABLE)) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Returning to Previous Mode Failure...");
    return MCP2515_FAIL;
  }
  dummy = mcp2515_readRegister(MCP_CANSTAT);
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "MCP_CANSTAT: %#04x", dummy);
  // if (dummy == 0) {
  //   mcp2515_setCANCTRL_Mode(MCP_NORMAL | CLKOUT_ENABLE);
  // }
  // rclcpp::sleep_for(std::chrono::milliseconds(1));
  return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_id(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
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

  // mcp2515_setRegisterS(mcp_addr, tbufdata, 4);

  mcp2515_setRegister(0x31, (canid >> 3) & 0XFF);
  mcp2515_setRegister(0x32, (canid & 0x07) << 5);
  mcp2515_setRegister(0x33, 0);
  mcp2515_setRegister(0x34, 0);
  // mcp2515_setRegister(TXB0DLC, len);
}

/*********************************************************************************************************
** Function name:           mcp2515_write_mf
** Descriptions:            Write Masks and Filters
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_mf(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
{
  uint16_t canid;
  uint8_t tbufdata[4];

  canid = (uint16_t)(id & 0x0FFFF);

  if (ext == 1) {
    tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (uint8_t)(canid & 0x03);
    tbufdata[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
    tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
    tbufdata[MCP_SIDH] = (uint8_t)(canid >> 5);
  } else {
    tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
    tbufdata[MCP_SIDH] = (uint8_t)(canid >> 3);
  }

  mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_id(const uint8_t mcp_addr, uint8_t * ext, uint32_t * id)
{
  uint8_t tbufdata[4];

  *ext = 0;
  *id = 0;

  mcp2515_readRegisterS(mcp_addr, tbufdata, 4);

  *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

  if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M) {
    /* extended id                  */
    *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
    *id = (*id << 8) + tbufdata[MCP_EID8];
    *id = (*id << 8) + tbufdata[MCP_EID0];
    *ext = 1;
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            Write message
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_canMsg(const uint8_t buffer_sidh_addr)
{
  uint8_t mcp_addr;

  mcp_addr = buffer_sidh_addr;
  mcp2515_setRegisterS(mcp_addr + 5, m_nDta, m_nDlc); /* write data bytes             */

  if (m_nRtr == 1) /* if RTR set bit in byte       */
  {
    m_nDlc |= MCP_RTR_MASK;
  }

  // mcp2515_setRegister((mcp_addr + 4), m_nDlc);  /* write the RTR and DLC        */
  mcp2515_setRegister(0x35, m_nDlc);            /* write the RTR and DLC        */
  mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID); /* write CAN id                 */
}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            Read message
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_canMsg(const uint8_t buffer_sidh_addr) /* read can msg                 */
{
  uint8_t ctrl;
  uint8_t mcp_addr;

  mcp_addr = buffer_sidh_addr;

  mcp2515_read_id(mcp_addr, &m_nExtFlg, &m_nID);

  ctrl = mcp2515_readRegister(mcp_addr - 1);
  m_nDlc = mcp2515_readRegister(mcp_addr + 4);

  if (ctrl & 0x08) {
    m_nRtr = 1;
  } else {
    m_nRtr = 0;
  }

  m_nDlc &= MCP_DLC_MASK;
  mcp2515_readRegisterS(mcp_addr + 5, &(m_nDta[0]), m_nDlc);
}

/*********************************************************************************************************
** Function name:           mcp2515_getNextFreeTXBuf
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_getNextFreeTXBuf(uint8_t * txbuf_address) /* get Next free txbuf */
{
  uint8_t i, ctrlval;
  uint8_t ctrlregs[MCP_N_TXBUFFERS] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};

  *txbuf_address = 0x00;

  /* check all 3 TX-Buffers       */
  for (i = 0; i < MCP_N_TXBUFFERS; i++) {
    ctrlval = mcp2515_readRegister(ctrlregs[i]);
    if ((ctrlval & MCP_TXB_TXREQ_M) == 0) {
      *txbuf_address = ctrlregs[i] + 1; /* return SIDH-address of Buffer*/
      return MCP2515_OK;                /* ! function exit */
    }
  }
  return MCP_ALLTXBUSY;
}

/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Public function to declare CAN class and the /CS pin.
*********************************************************************************************************/
// void MCP_CAN::init_Para(int spi_channel, int spi_baudrate, uint8_t gpio_can_interrupt, uint8_t
// gpio_can_cs)
// {
//     this->spi_channel        = spi_channel;
//     this->spi_baudrate       = spi_baudrate;
//     this->gpio_can_interrupt = gpio_can_interrupt;
//     this->gpio_can_cs = gpio_can_cs;

//     delay_spi_can.tv_sec  = 0;
//     delay_spi_can.tv_nsec = 5000L; // wait 5 microseconds between 2 spi transfers
// }

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
uint8_t MCP_CAN::initCAN(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
  uint8_t res;

  res = initMCP2515(idmodeset, speedset, clockset);
  if (res == MCP2515_OK) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "CAN is initialized");
    return CAN_OK;
  }

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "CAN initializtion is failed");
  return CAN_FAILINIT;
}

// uint8_t MCP_CAN::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
// {
//     uint8_t res;

//     res = initMCP2515(idmodeset, speedset, clockset);
//     if (res == MCP2515_OK)
//     {
//         return CAN_OK;
//     }

//     return CAN_FAILINIT;
// }

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Mask(uint8_t num, uint8_t ext, uint32_t ulData)
{
  uint8_t res = MCP2515_OK;

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Starting to Set Mask!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering Configuration Mode Failure...");
    return res;
  }

  if (num == 0) {
    mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);
  } else if (num == 1) {
    mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
  } else {
    res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(this->mcpMode);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering Previous Mode Failure...");
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Mask Failure...");
    return res;
  }
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Mask Successful!");
  return res;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Mask(uint8_t num, uint32_t ulData)
{
  uint8_t res = MCP2515_OK;
  uint8_t ext = 0;

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Starting to Set Mask!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Enter Configuration Mode Failure...");
    return res;
  }

  if ((num & 0x80000000) == 0x80000000) {
    ext = 1;
  }

  if (num == 0) {
    mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);
  } else if (num == 1) {
    mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
  } else {
    res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(this->mcpMode);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering Previous Mode Failure...");
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Mask Failure...");
    return res;
  }
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Mask Successful!");
  return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Filt(uint8_t num, uint8_t ext, uint32_t ulData)
{
  uint8_t res = MCP2515_OK;

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Starting to Set Filter!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Enter Configuration Mode Failure...");
    return res;
  }

  switch (num) {
    case 0:
      mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
      break;

    case 1:
      mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
      break;

    case 2:
      mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
      break;

    case 3:
      mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
      break;

    case 4:
      mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
      break;

    case 5:
      mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
      break;

    default:
      res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(this->mcpMode);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering Previous Mode Failure...");
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Filter Failure...");
    return res;
  }
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Filter Successfull!");
  return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Filt(uint8_t num, uint32_t ulData)
{
  uint8_t res = MCP2515_OK;
  uint8_t ext = 0;

  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Starting to Set Filter!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Enter Configuration Mode Failure...");
    return res;
  }

  if ((num & 0x80000000) == 0x80000000) {
    ext = 1;
  }

  switch (num) {
    case 0:
      mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
      break;

    case 1:
      mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
      break;

    case 2:
      mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
      break;

    case 3:
      mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
      break;

    case 4:
      mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
      break;

    case 5:
      mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
      break;

    default:
      res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(this->mcpMode);
  if (res > 0) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Entering Previous Mode Failure...");
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Filter Failure...");
    return res;
  }
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "Setting Filter Successful!");
  return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
uint8_t MCP_CAN::setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * pData)
{
  int i = 0;

  m_nID = id;
  m_nRtr = rtr;
  m_nExtFlg = ext;
  m_nDlc = len;
  for (i = 0; i < MAX_CHAR_IN_MESSAGE; i++) {
    m_nDta[i] = *(pData + i);
  }

  return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
uint8_t MCP_CAN::clearMsg()
{
  m_nID = 0;
  m_nDlc = 0;
  m_nExtFlg = 0;
  m_nRtr = 0;
  m_nfilhit = 0;
  for (int i = 0; i < m_nDlc; i++) {
    m_nDta[i] = 0x00;
  }

  return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP_CAN::sendMsg()
{
  uint8_t res, res1, txbuf_n;
  uint16_t uiTimeOut = 0;

  do {
    res = mcp2515_getNextFreeTXBuf(&txbuf_n); /* info = addr.                 */
    uiTimeOut++;
  } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

  if (uiTimeOut == TIMEOUTVALUE) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "Get MCP2515 Tx Buff Time Out!");
    return CAN_GETTXBFTIMEOUT; /* get tx buff time out         */
  }
  uiTimeOut = 0;
  mcp2515_write_canMsg(txbuf_n);
  mcp2515_modifyRegister(txbuf_n - 1, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);

  do {
    uiTimeOut++;
    res1 = mcp2515_readRegister(txbuf_n - 1); /* read send buff ctrl reg  */
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
void MCP_CAN::mcp2515_send(uint32_t canid, uint8_t * buf, uint8_t len)
{
  // uint8_t tempdata = MCP2515_ReadByte(CAN_RD_STATUS);
  uint8_t dly = 0;
  while ((mcp2515_readRegister(MCP_TXB0CTRL) & 0x08) && (dly < 10)) {
    rclcpp::sleep_for(std::chrono::milliseconds(2));
    dly++;
  }

  mcp2515_setRegister(0x31, (canid >> 3) & 0XFF);
  mcp2515_setRegister(0x32, (canid & 0x07) << 5);
  // mcp2515_setRegister(0x33, 0);
  // mcp2515_setRegister(0x34, 0);

  // for (uint8_t j = 0; j < 8; j++) {
  //   mcp2515_setRegister(0x36 + j, buf[j]);
  // }
  mcp2515_setRegister(0x36, 0x01);
  mcp2515_setRegister(0x37, 0x02);
  mcp2515_setRegister(0x38, 0x03);
  mcp2515_setRegister(0x39, 0x04);
  mcp2515_setRegister(0x3a, 0x05);
  mcp2515_setRegister(0x3b, 0x06);
  mcp2515_setRegister(0x3c, 0x07);
  mcp2515_setRegister(0x3d, 0x08);
  mcp2515_setRegister(0x35, 0x08);
  mcp2515_setRegister(MCP_TXB0CTRL, 0x08);
}

uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t * buf)
{
  uint8_t res;

  setMsg(id, 0, ext, len, buf);
  res = sendMsg();

  return res;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t len, uint8_t * buf)
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
uint8_t MCP_CAN::readMsg()
{
  uint8_t stat, res;

  stat = mcp2515_readStatus();

  if (stat & MCP_STAT_RX0IF) /* Msg in Buffer 0              */
  {
    mcp2515_read_canMsg(MCP_RXBUF_0);
    mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
    res = CAN_OK;
  } else if (stat & MCP_STAT_RX1IF) /* Msg in Buffer 1              */
  {
    mcp2515_read_canMsg(MCP_RXBUF_1);
    mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
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
uint8_t MCP_CAN::readMsgBuf(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t buf[])
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
uint8_t MCP_CAN::readMsgBuf(uint32_t * id, uint8_t * len, uint8_t buf[])
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

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            Public function, Checks for received data.  (Used if not using the
*interrupt output)
*********************************************************************************************************/
uint8_t MCP_CAN::checkReceive(void)
{
  uint8_t res;

  res = mcp2515_readStatus(); /* RXnIF in Bit 1 and 0         */
  if (res & MCP_STAT_RXIF_MASK) {
    return CAN_MSGAVAIL;
  } else {
    return CAN_NOMSG;
  }
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            Public function, Returns error register data.
*********************************************************************************************************/
uint8_t MCP_CAN::checkError(void)
{
  uint8_t eflg = mcp2515_readRegister(MCP_EFLG);

  if (eflg & MCP_EFLG_ERRORMASK) {
    return CAN_CTRLERROR;
  } else {
    return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           getError
** Descriptions:            Returns error register value.
*********************************************************************************************************/
uint8_t MCP_CAN::getError(void)
{
  return mcp2515_readRegister(MCP_EFLG);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountRX
** Descriptions:            Returns REC register value
*********************************************************************************************************/
uint8_t MCP_CAN::errorCountRX(void)
{
  return mcp2515_readRegister(MCP_REC);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountTX
** Descriptions:            Returns TEC register value
*********************************************************************************************************/
uint8_t MCP_CAN::errorCountTX(void)
{
  return mcp2515_readRegister(MCP_TEC);
}

/*********************************************************************************************************
** Function name:           mcp2515_enOneShotTX
** Descriptions:            Enables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP_CAN::enOneShotTX(void)
{
  uint8_t value;
  mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
  value = mcp2515_readRegister(MCP_CANCTRL);
  if ((value & MODE_ONESHOT) != MODE_ONESHOT) {
    return CAN_FAIL;
  } else {
    return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_disOneShotTX
** Descriptions:            Disables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP_CAN::disOneShotTX(void)
{
  uint8_t value;
  mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
  value = mcp2515_readRegister(MCP_CANCTRL);
  if ((value & MODE_ONESHOT) != 0) {
    return CAN_FAIL;
  } else {
    return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           startCharging
** Descriptions:            Starts charging at voltage and current specified
*********************************************************************************************************/
uint8_t MCP_CAN::queryCharger(float voltage, float current, int address, int charge)
{
  uint16_t v = static_cast<uint16_t>(voltage * 10);
  uint16_t i = static_cast<uint16_t>(current * 10);
  uint8_t messageCharger[5] = {
    static_cast<uint8_t>((v >> 8) & 0xFF), static_cast<uint8_t>(v & 0xFF),
    static_cast<uint8_t>((i >> 8) & 0xFF), static_cast<uint8_t>(i & 0xFF),
    static_cast<uint8_t>(charge)};  // Los 5 bytes que enviamos al Cargador

  int res = sendMsgBuf(static_cast<uint8_t>(address), 1, 5, messageCharger);

  return res;
}

/*********************************************************************************************************
** Function name:           queryBMS
** Descriptions:            asks BMS for voltage and sends shunt voltage
*********************************************************************************************************/
uint8_t MCP_CAN::queryBMS(int moduleID, int shuntVoltageMillivolts)
{
  uint8_t messageBMS[2] = {
    static_cast<uint8_t>((shuntVoltageMillivolts >> 8) & 0xFF),
    static_cast<uint8_t>(shuntVoltageMillivolts & 0xFF)};  // Los 2 bytes que enviamos al BMS

  int res = sendMsgBuf(300 + 10 * moduleID, 1, 2, messageBMS);

  return res;
}

}  // namespace amp
