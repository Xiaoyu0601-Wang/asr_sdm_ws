#ifndef CAN_PROTOCOL_HPP_
#define CAN_PROTOCOL_HPP_

#include <iostream>
#include <memory>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <vector>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>

#include "usr_sdm_controller/mcp_can_rpi.hpp"

namespace amp
{

class CANProtocol
{
  protected:
    const int SPI_CHANNEL = 0;
    const uint8_t SPI_CS_PIN = 8;
    const uint8_t SPI_INT_PIN = 25;
    const uint8_t CANSpeed = CAN_500KBPS;
    const uint8_t MCPClock = MCP_12MHZ;
    const uint8_t MCPMode = MCP_NORMAL;

    //CAN
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
//    const uint8_t CAN_EXT = 1;

    void writeCANMsg(uint16_t screwUnitID, std::vector<uint8_t> &msg);
    void readCANMsg(uint16_t screwUnitID, std::vector<uint8_t> *msg);

//  public:
//    MCP_CAN::Ptr CAN;

//    void startCAN(void);
//    void endCAN(void);

  public:
	CANProtocol();
    virtual ~CANProtocol();

//    static MCP_CAN CAN;

    void interfaceSetup(void);
//    static void printCANMsg();

    typedef std::unique_ptr<CANProtocol> Ptr;
};

} //amp

#endif /* CAN_PROTOCOL_HPP_ */
