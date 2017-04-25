
#pragma once

class CANService;

#include "N2kMsg.h"

#include <stdint.h>
#include <vector>

class CANFrameReceiver
{

public:
	CANFrameReceiver(CANService& service, std::vector<uint32_t> IDs);
	CANFrameReceiver(CANService& service, uint32_t ID);

	//just change to:
	// virtual void process(CanMsg& cmsg, N2kMsg& nmsg) = 0;
	//to avoid the need for another receiver class
	virtual void processPGN(CanMsg& msg) = 0;
};
