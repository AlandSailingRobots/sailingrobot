
#ifndef canpgnreceiver_h
#define canpgnreceiver_h

class CANService;

#include <stdint.h>
#include <vector>

class CANPGNReceiver
{

public:
	CANPGNReceiver(CANService& service, std::vector<uint32_t> PGNs);
	CANPGNReceiver(CANService& service, uint32_t PGN);

	virtual void processPGN(std::vector<uint8_t> Data, uint32_t PGN) = 0;
};

#endif
