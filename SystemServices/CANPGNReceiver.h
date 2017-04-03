
#ifndef canpgnreciever_h
#define canpgnreciever_h

class CANService;

#include "CANService.h"

#include <stdint.h>
#include <vector>

class CanPGNReceiver
{

public:
	CanPGNReceiver(CANService& service) : m_Service(service)
	{ };

	virtual void processPGN(std::vector<uint8_t> Data, uint32_t PGN) = 0;

private:
	CANService& m_Service;

};

#endif
