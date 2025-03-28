#ifndef UART_CAN_HPP_
#define UART_CAN_HPP_

/* System headers */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>

/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>

/* c-periphery headers */
#include "periphery/serial.h"

#define FRAME_HEAD_LENGTH 4
#define FRAME_TAIL_LENGTH 1
#define MAX_FRAME_DATA_LENGTH 8

namespace amp
{
struct UartFrame
{
  uint8_t m_nExtFlg;  // Identifier Type
                      // Extended (29 bit) or Standard (11 bit)
  uint32_t m_nID;     // CAN ID
  uint8_t m_nDlc;     // Data Length Code
  uint8_t m_nDta[FRAME_HEAD_LENGTH + MAX_FRAME_DATA_LENGTH];  // Data array
  uint8_t m_nRtr;                                             // Remote request flag
  uint8_t m_nfilhit;  // The number of the filter that matched the message
  uint8_t mcpMode;    // Mode to return to after configurations are performed.
  uint8_t frame_head;
  uint8_t frame_tail;
};

class UART_CAN
{
public:
  UART_CAN()
  {
    serial_ = serial_new();

    /* Open /dev/ttyS3 with baudrate 115200, and defaults of 8N1, no flow control */
    if (serial_open(serial_, "/dev/ttyS3", 115200) < 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("hardware"), "serial_ttyS3_open(): %s", serial_errmsg(serial_));
    }

    uart_frame_.frame_head = 0xAA;
    uart_frame_.frame_tail = 0xFF;
  }

  ~UART_CAN()
  {
    serial_close(serial_);
    serial_free(serial_);
  }

private:
  int spi_channel;
  int spi_baudrate;

  serial_t * serial_;
  UartFrame uart_frame_;

  /*********************************************************************************************************
   *  uart driver function
   *********************************************************************************************************/
  // private:
private:
  struct timespec delay_spi_can = {0, 0L};

  bool spiTransfer(uint8_t byte_number, unsigned char * tx_buf, unsigned char * rx_buf);

  bool resetuart(void);  // Soft Reset uart

  uint8_t uart_readRegister(const uint8_t address);  // Read uart register

  void uart_readRegisterS(
    const uint8_t address,  // Read uart successive registers
    uint8_t values[], const uint8_t n);

  void uart_setRegister(
    const uint8_t address,  // Set uart register
    const uint8_t value);

  void uart_setRegisterS(
    const uint8_t address,  // Set uart successive registers
    const uint8_t values[], const uint8_t n);

  void uart_initCANBuffers(void);

  void uart_modifyRegister(
    const uint8_t address,  // Set specific bit(s) of a register
    const uint8_t mask, const uint8_t data);

  uint8_t uart_readStatus(void);                        // Read uart Status
  uint8_t uart_setCANCTRL_Mode(const uint8_t newmode);  // Set mode
  uint8_t uart_configRate(
    const uint8_t canSpeed,  // Set baudrate
    const uint8_t canClock);

  uint8_t inituart(
    const uint8_t canIDMode,  // Initialize Controller
    const uint8_t canSpeed, const uint8_t canClock);

  void uart_write_mf(
    const uint8_t mcp_addr,  // Write CAN Mask or Filter
    const uint8_t ext, const uint32_t id);

  void uart_write_id(
    const uint8_t mcp_addr,  // Write CAN ID
    const uint8_t ext, const uint32_t id);

  void uart_read_id(
    const uint8_t mcp_addr,  // Read CAN ID
    uint8_t * ext, uint32_t * id);

  void uart_write_canMsg(const uint8_t buffer_sidh_addr);  // Write CAN message
  void uart_read_canMsg(const uint8_t buffer_sidh_addr);   // Read CAN message
  uint8_t uart_getNextFreeTXBuf(uint8_t * txbuf_n);        // Find empty transmit buffer

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
  uint8_t initUART2CAN(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);
  // uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);     // Initilize
  // controller prameters
  uint8_t setMode(uint8_t opMode);  // Set operational mode
  void uart_send(uint32_t canid, uint8_t * buf, uint8_t len);
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

  typedef std::unique_ptr<UART_CAN> Ptr;
};

}  // namespace amp

#endif
