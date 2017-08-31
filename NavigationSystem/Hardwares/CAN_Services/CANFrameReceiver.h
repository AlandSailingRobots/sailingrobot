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

	virtual void processFrame(CanMsg& msg) = 0;
};
