#include "asr_sdm_controller/can_protocol.hpp"

//MCP2515 can0;
//struct can_frame rx;

namespace amp
{

//MCP_CAN::Ptr CAN;
//
//void printCANMsg()
//{
//    uint8_t  len    = 0;
//    uint8_t  buf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
//    uint32_t canId  = 0;
//
//    printf("CAN Interrupt Triggered\n");
//    if (CAN_MSGAVAIL == CAN->checkReceive())  // check if data coming
//    {
//        // uint8_t MCP_CAN::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t buf[])
//        // Read data rellena canId, len y guarda los datos en buf
//        CAN->readMsgBuf(&canId, &len, &buf[0]);
//
//        canId = canId & 0x1FFFFFFF;
//
//        printf("-----------------------------\n");
//        printf("Received data from ID: %u | len:%d\n", canId, len);
//
//        for (int i = 0; i < len; i++) // print the data
//        {
//            printf("(%d)", buf[i]);
//            printf("\t");
//        }
//    }
//}

CANProtocol::CANProtocol()
{
	/**
	 * CAN
	 * */
	memset(&frame, 0, sizeof(struct can_frame));
	system("sudo ip link set can0 up type can bitrate 500000");

    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
//        return 1;
    }

    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
//        return 1;
    }

	//3.Bind the socket to can0
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
		perror("bind failed");
//		return 1;
	}

	printf("CAN is initialized.\n");
}

CANProtocol::~CANProtocol()
{
	/**
	 * CAN
	 * */
	//7.Close the socket and can0
	close(s);
	system("sudo ifconfig can0 down");
	printf("CAN is closed.\n");
}

void CANProtocol::interfaceSetup(void)
{
	std::vector<uint8_t> msg;
	for(uint8_t i = 0; i < 8; i++)
	{
		msg.push_back(i);
	}
	writeCANMsg(1, msg);
//	CAN.reset(new MCP_CAN);
//
//	CAN->init_Para(SPI_CHANNEL, 2000000, SPI_INT_PIN, SPI_CS_PIN);

//	wiringPiSetup();
//    int result = wiringPiSetupGpio();
//    if (!result)
//    {
//        printf("Gpio started\n");
//    }
//    else
//    {
//        printf("Gpio startup fail\n");
//    }
//
//    pinMode(SPI_INT_PIN, INPUT);
//    wiringPiSetupGpio();
//    pinMode(SPI_CS_PIN, OUTPUT);//(8, OUTPUT);//
//    digitalWrite(SPI_CS_PIN, LOW);//(8, LOW);//HIGH
////    wiringPiSetupGpio();
//
//    struct timespec req;
//    req.tv_sec = 0;        // seconds
//    req.tv_nsec = 500000L; // nanoseconds
//    nanosleep(&req, nullptr);
//
//	if (wiringPiSPISetup(SPI_CHANNEL, 2000000) < 0)
//	{
//		fprintf(stderr, "Can't open the SPI bus: %s\n", strerror(errno));
//		exit(EXIT_FAILURE);
//	}
//
//    nanosleep(&req, nullptr);
//    printf("GPIO Pins initialized & SPI started\n");
//
//	// Attach interrupt to read incoming messages
//	wiringPiISR(SPI_INT_PIN, INT_EDGE_FALLING, printCANMsg);
/////////////////////////////////////////////////////////////
//	CAN->setupInterruptGpio();
//    CAN->setupSpi();
//    printf("GPIO Pins initialized & SPI started\n");

    // Attach interrupt to read incoming messages
//    wiringPiISR(SPI_INT_PIN, INT_EDGE_FALLING, printCANMsg);

    /* Start CAN bus
     * uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);
     */

//    while (CAN_OK != CAN->begin(MCP_ANY, CANSpeed, MCPClock))
//    {
//        printf("CAN BUS Shield init fail\n");
//        printf("Trying to init CAN BUS Shield again\n\n");
//
//        struct timespec req;
//        req.tv_sec = 1;        // seconds
//        req.tv_nsec = 0; // nanoseconds
//        nanosleep(&req, nullptr);
//    }
//    printf("CAN BUS Shield init ok!\n");
//    CAN->setMode(MCPMode);
//
//    uint8_t  data[8] = { 3, 14, 15, 2, 1, 2, 3, 4 };
//    printf("\n\nMessage sent: %d\n", CAN->sendMsgBuf(5, 0, 8, data));
}

//void CANProtocol::startCAN()
//{
//	digitalWrite(SPI_CS_PIN, LOW);
//    std::cout << "Start CAN Interface" << std::endl;
//}
//
//void CANProtocol::endCAN()
//{
//	digitalWrite(SPI_CS_PIN, HIGH);
//    std::cout << "End CAN Interface" << std::endl;
//}

void CANProtocol::writeCANMsg(uint16_t screwUnitID, std::vector<uint8_t> &msg)
{
	//4.Disable filtering rules, do not receive packets, only send
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	//5.Set send data
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
	for(int i = 0; i < 8; i++)
	{
		printf("data[%d] = %d\r\n", i, frame.data[i]);
	}

	//6.Send message
	nbytes = write(s, &frame, sizeof(frame));
	if(nbytes != sizeof(frame))
	{
		printf("Send Error frame[0]!\r\n");
	}
}

void CANProtocol::readCANMsg(uint16_t screwUnitID, std::vector<uint8_t> *msg)
{
    //4.Define receive rules
    struct can_filter rfilter[1];
    rfilter[0].can_id = 0x123;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
}

} //amp

//int main() {
//    stdio_init_all();
//
//    //Initialize interface
//    can0.reset();
//    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
//    can0.setNormalMode();
//
//    //Listen loop
//    while(true) {
//        if(can0.readMessage(&rx) == MCP2515::ERROR_OK) {
//            printf("New frame from ID: %10x\n", rx.can_id);
//        }
//    }
//
//    return 0;
//}
