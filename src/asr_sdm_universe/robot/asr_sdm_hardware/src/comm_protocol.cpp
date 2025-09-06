#include "asr_sdm_hardware/comm_protocol.hpp"

namespace amp
{
CommProtocol::CommProtocol()
{
  /**
   * CAN
   * */
  memset(&frame, 0, sizeof(struct can_frame));
  system("sudo ip link set can0 up type can bitrate 500000");

  // 1.Create socket
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    perror("socket PF_CAN failed");
    //        return 1;
  }

  // 2.Specify can0 device
  strcpy(ifr.ifr_name, "can0");
  ret = ioctl(s, SIOCGIFINDEX, &ifr);
  if (ret < 0) {
    perror("ioctl failed");
    //        return 1;
  }

  // 3.Bind the socket to can0
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    perror("bind failed");
    //		return 1;
  }

  printf("CAN is initialized.\n");
}

CommProtocol::~CommProtocol()
{
  close(s);
  system("sudo ifconfig can0 down");
  printf("CAN is closed.\n");
}

void CommProtocol::interfaceSetup(void)
{
  std::vector<uint8_t> msg;
  for (uint8_t i = 0; i < 8; i++) {
    msg.push_back(i);
  }
  writeCANMsg(1, msg);
}

void CommProtocol::writeCANMsg(uint16_t screwUnitID, std::vector<uint8_t> & msg)
{
  // 4.Disable filtering rules, do not receive packets, only send
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  // 5.Set send data
  frame.can_id = screwUnitID;
  frame.can_dlc = 8;
  frame.data[0] = msg[0];
  frame.data[1] = msg[1];
  frame.data[2] = msg[2];
  frame.data[3] = msg[3];
  frame.data[4] = msg[4];
  frame.data[5] = msg[5];
  frame.data[6] = msg[6];
  frame.data[7] = msg[7];

  printf("can_id  = 0x%X\r\n", frame.can_id);
  for (int i = 0; i < 8; i++) {
    printf("data[%d] = %d\r\n", i, frame.data[i]);
  }

  // 6.Send message
  nbytes = write(s, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {
    printf("Send Error frame[0]!\r\n");
  }
}

void CommProtocol::readCANMsg(uint16_t screwUnitID, std::vector<uint8_t> * msg)
{
  // 4.Define receive rules
  struct can_filter rfilter[1];
  rfilter[0].can_id = 0x123;
  rfilter[0].can_mask = CAN_SFF_MASK;
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
}

}  // namespace amp
