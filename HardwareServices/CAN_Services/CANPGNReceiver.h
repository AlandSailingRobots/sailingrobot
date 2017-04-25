
#ifndef canpgnreceiver_h
#define canpgnreceiver_h

class CANService;

#include "N2kMsg.h"

#include <stdint.h>
#include <vector>

class CANPGNReceiver
{

public:
	CANPGNReceiver(CANService& service, std::vector<uint32_t> PGNs);
	CANPGNReceiver(CANService& service, uint32_t PGN);

	//just change to:
	// virtual void process(CanMsg& cmsg, N2kMsg& nmsg) = 0;
	//to avoid the need for another receiver class
	virtual void processPGN(N2kMsg& msg) = 0;
};

#endif
