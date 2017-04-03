
#ifndef canpgnreceiver_h
#define canpgnreceiver_h

#include <stdint.h>
#include <vector>

class CANPGNReceiver
{

public:
	virtual void processPGN(std::vector<uint8_t> Data, uint32_t PGN) = 0;
};

#endif