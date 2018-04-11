#ifndef canbus__h
#define canbus__h

#include <SPI.h>

#include "mcp2515.h"
#include "MsgParsing.h"


//typedef std::tuple<uint32_t,uint8_t> IDsID;



class CanbusClass
{
public:
	CanbusClass() {};
	bool Init(int chipSelectPin);
	uint8_t CheckForMessages();

	bool SendMessage(CanMsg *Msg);
	bool GetMessage(CanMsg *Msg);


	void SetNormalMode();
	void SetSleepMode();
	void SetLoopBackMode();
	void SetListenOnlyMode();
	void SetConfigMode();
	void SetMasksAndActivateFilters(uint32_t Mask1, uint32_t Mask2, bool RollOver);
	void SetFilter(int FilterIndex, uint32_t Filter);
	void SetFilterAndMask(int ReceiveBuffer, int FilterIndex, uint32_t Filter, uint32_t Mask);
};

#endif
