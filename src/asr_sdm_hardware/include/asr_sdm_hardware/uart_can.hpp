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
  UART_CAN(const std::string & uart_port, uint32_t uart_baudrate);

  ~UART_CAN();

private:
  int spi_channel;
  int spi_baudrate;

  serial_t * serial_;
  UartFrame uart_frame_;
  const char * uart_port_;

  /*********************************************************************************************************
   *  uart driver function
   *********************************************************************************************************/
  // private:
private:
  struct timespec delay_spi_can = {0, 0L};

  bool uartTransfer(uint8_t byte_number, unsigned char * tx_buf);

  void uart_readRegisterS(
    const uint8_t address,  // Read uart successive registers
    uint8_t values[], const uint8_t n);

  void uart_setRegister(
    const uint8_t address,  // Set uart register
    const uint8_t value);

  void uart_setRegisterS(
    const uint8_t address,  // Set uart successive registers
    const uint8_t values[], const uint8_t n);

  uint8_t uart_readStatus(void);                        // Read uart Status
  uint8_t uart_setCANCTRL_Mode(const uint8_t newmode);  // Set mode
  uint8_t uart_configRate(
    const uint8_t canSpeed,  // Set baudrate
    const uint8_t canClock);

  void uart_write_id(
    const uint8_t mcp_addr,  // Write CAN ID
    const uint8_t ext, const uint32_t id);

  void uart_read_id(
    const uint8_t mcp_addr,  // Read CAN ID
    uint8_t * ext, uint32_t * id);

  /*********************************************************************************************************
   *  CAN operator function
   *********************************************************************************************************/
  uint8_t setMsg(
    uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * pData);  // Set message
  uint8_t clearMsg();  // Clear all message to zero
  uint8_t readMsg();   // Read message
  uint8_t sendMsg();   // Send message

public:
  uint8_t initUART2CAN(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);
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

  typedef std::unique_ptr<UART_CAN> Ptr;
};

}  // namespace amp

#endif
