#ifndef UART2CAN_HPP_
#define UART2CAN_HPP_

/* System headers */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>

/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_hardware_msgs/msg/can_frame.hpp"

/* c-periphery headers */
#include "periphery/serial.h"

#define FRAME_HEAD_LENGTH 4
#define FRAME_TAIL_LENGTH 1
#define MAX_FRAME_DATA_LENGTH 8

namespace amp
{
struct Uart2CanFrame
{
  uint8_t m_nExtFlg;  // Identifier Type
                      // Extended (29 bit) or Standard (11 bit)
  uint32_t m_nID;     // CAN ID
  uint8_t m_nDlc;     // Data Length Code
  uint8_t tx_buf[8];
  uint8_t rx_buf[8];
  uint8_t m_nDta[FRAME_HEAD_LENGTH + MAX_FRAME_DATA_LENGTH];
  uint8_t m_nRtr;
  uint8_t frame_head;
  uint8_t frame_tail;
};

class UART_CAN
{
public:
  UART_CAN(const std::string & uart_port, uint32_t uart_baudrate);

  ~UART_CAN();

private:
  serial_t * serial_;
  Uart2CanFrame uart2can_frame_;
  const char * uart_port_;

  /*************************************************************
   *  uart driver function
   *************************************************************/
private:
  bool uartTransfer(uint8_t byte_number, unsigned char * tx_buf);
  uint8_t uart_configRate(const uint8_t canSpeed, const uint8_t canClock);
  void uart_write_id(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id);
  void uart_read_id(const uint8_t mcp_addr, uint8_t * ext, uint32_t * id);

  /*************************************************************
   *  CAN operator function
   *************************************************************/
  uint8_t setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * pData);
  uint8_t clearMsg();

public:
  void uart_send(uint32_t canid, uint8_t * buf, uint8_t len);
  uint8_t sendMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t * buf);
  uint8_t readMsg(uint32_t * id, uint8_t * ext, uint8_t * len, uint8_t * buf);

  typedef std::unique_ptr<UART_CAN> Ptr;
};

}  // namespace amp

#endif
