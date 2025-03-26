#ifndef MCP_CAN_HPP_
#define MCP_CAN_HPP_
/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>

/* c-periphery headers */
#include "asr_sdm_controller/mcp_can_dfs.h"
#include "periphery/spi.h"

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>  // For malloc and free
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>

#define MAX_CHAR_IN_MESSAGE 8

#define CAN_MODEL_NUMBER 10000

#define SPI_PORT 3
#define SPI_CS 0
#define SPI_FREQUENCY 1000000

namespace amp
{

class MCP_CAN
{
public:
  MCP_CAN()
  {
    spi_ = spi_new();
    /* Open spidev3.0 with mode 0 and max speed 1MHz */
    if (spi_open(spi_, "/dev/spidev3.0", 0, 1000000) < 0) {
      fprintf(stderr, "spi_open(): %s\n", spi_errmsg(spi_));
    }
  }

  ~MCP_CAN()
  {
    spi_close(spi_);
    spi_free(spi_);
  }

private:
  uint8_t m_nExtFlg;                    // Identifier Type
                                        // Extended (29 bit) or Standard (11 bit)
  uint32_t m_nID;                       // CAN ID
  uint8_t m_nDlc;                       // Data Length Code
  uint8_t m_nDta[MAX_CHAR_IN_MESSAGE];  // Data array
  uint8_t m_nRtr;                       // Remote request flag
  uint8_t m_nfilhit;                    // The number of the filter that matched the message
  uint8_t mcpMode;                      // Mode to return to after configurations are performed.

  int spi_channel;
  int spi_baudrate;
  uint8_t gpio_can_interrupt;
  uint8_t gpio_can_cs;

  spi_t * spi_;

  /*********************************************************************************************************
   *  mcp2515 driver function
   *********************************************************************************************************/
  // private:
private:
  struct timespec delay_spi_can = {0, 0L};

  bool spiTransfer(uint8_t byte_number, unsigned char * tx_buf, unsigned char * rx_buf);

  bool resetMCP2515(void);  // Soft Reset MCP2515

  uint8_t mcp2515_readRegister(const uint8_t address);  // Read MCP2515 register

  void mcp2515_readRegisterS(
    const uint8_t address,  // Read MCP2515 successive registers
    uint8_t values[], const uint8_t n);

  void mcp2515_setRegister(
    const uint8_t address,  // Set MCP2515 register
    const uint8_t value);

  void mcp2515_setRegisterS(
    const uint8_t address,  // Set MCP2515 successive registers
    const uint8_t values[], const uint8_t n);

  void mcp2515_initCANBuffers(void);

  void mcp2515_modifyRegister(
    const uint8_t address,  // Set specific bit(s) of a register
    const uint8_t mask, const uint8_t data);

  uint8_t mcp2515_readStatus(void);                        // Read MCP2515 Status
  uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode);  // Set mode
  uint8_t mcp2515_configRate(
    const uint8_t canSpeed,  // Set baudrate
    const uint8_t canClock);

  uint8_t initMCP2515(
    const uint8_t canIDMode,  // Initialize Controller
    const uint8_t canSpeed, const uint8_t canClock);

  void mcp2515_write_mf(
    const uint8_t mcp_addr,  // Write CAN Mask or Filter
    const uint8_t ext, const uint32_t id);

  void mcp2515_write_id(
    const uint8_t mcp_addr,  // Write CAN ID
    const uint8_t ext, const uint32_t id);

  void mcp2515_read_id(
    const uint8_t mcp_addr,  // Read CAN ID
    uint8_t * ext, uint32_t * id);

  void mcp2515_write_canMsg(const uint8_t buffer_sidh_addr);  // Write CAN message
  void mcp2515_read_canMsg(const uint8_t buffer_sidh_addr);   // Read CAN message
  uint8_t mcp2515_getNextFreeTXBuf(uint8_t * txbuf_n);        // Find empty transmit buffer

  /*********************************************************************************************************
   *  CAN operator function
   *********************************************************************************************************/

  uint8_t setMsg(
    uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * pData);  // Set message
  uint8_t clearMsg();  // Clear all message to zero
  uint8_t readMsg();   // Read message
  uint8_t sendMsg();   // Send message

public:
  // void init_Para(int spi_channel, int spi_baudrate, uint8_t gpio_can_interrupt, uint8_t
  // gpio_can_cs);
  uint8_t initCAN(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);
  // uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);     // Initilize
  // controller prameters
  uint8_t init_Mask(uint8_t num, uint8_t ext, uint32_t ulData);  // Initilize Mask(s)
  uint8_t init_Mask(uint8_t num, uint32_t ulData);               // Initilize Mask(s)
  uint8_t init_Filt(uint8_t num, uint8_t ext, uint32_t ulData);  // Initilize Filter(s)
  uint8_t init_Filt(uint8_t num, uint32_t ulData);               // Initilize Filter(s)
  uint8_t setMode(uint8_t opMode);                               // Set operational mode
  void mcp2515_send(uint32_t canid, uint8_t * buf, uint8_t len);
  uint8_t sendMsgBuf(
    uint32_t id, uint8_t ext, uint8_t len, uint8_t * buf);      // Send message to transmit buffer
  uint8_t sendMsgBuf(uint32_t id, uint8_t len, uint8_t * buf);  // Send message to transmit buffer
  uint8_t readMsgBuf(
    uint32_t * id, uint8_t * ext, uint8_t * len,
    uint8_t * buf);  // Read message from receive buffer
  uint8_t readMsgBuf(
    uint32_t * id, uint8_t * len, uint8_t * buf);  // Read message from receive buffer
  uint8_t checkReceive(void);                      // Check for received data
  uint8_t checkError(void);                        // Check for errors
  uint8_t getError(void);                          // Check for errors
  uint8_t errorCountRX(void);                      // Get error count
  uint8_t errorCountTX(void);                      // Get error count
  uint8_t enOneShotTX(void);                       // Enable one-shot transmission
  uint8_t disOneShotTX(void);                      // Disable one-shot transmission

  uint8_t queryCharger(float voltage, float current, int address, int charge);  // Start charging
  uint8_t queryBMS(int moduleID, int shuntVoltageMillivolts);                   // Query BMS

  // bool setupInterruptGpio();
  // bool setupSpi();
  // bool canReadData();

  typedef std::unique_ptr<MCP_CAN> Ptr;
};

}  // namespace amp

#endif
