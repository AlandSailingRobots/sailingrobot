
#include "Canbus.h"
#include "mcp2515.h"

bool CanbusClass::SendMessage(CanMsg *Msg)
{
	return MCP2515_SendMessage(Msg, 0);
}
bool CanbusClass::GetMessage(CanMsg *Msg)
{
	return MCP2515_GetMessage(Msg, 0);
}
uint8_t CanbusClass::CheckForMessages()			//returns address of messge if there is one, otherwise returns 0
{
	return(MCP2515_CheckMessage());
}
bool CanbusClass::Init(int chipSelectPin)
{
	SPI.begin();
	pinMode(chipSelectPin, OUTPUT);
	return MCP2515_Init(chipSelectPin);
}

void CanbusClass::SetNormalMode()
{
	uint8_t Mode = 0x00;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}
void CanbusClass::SetSleepMode()
{
	uint8_t Mode = 0x20;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}
void CanbusClass::SetLoopBackMode()
{
	uint8_t Mode = 0x40;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}
void CanbusClass::SetListenOnlyMode()
{
	uint8_t Mode = 0x60;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}
void CanbusClass::SetConfigMode()
{
	uint8_t Mode = 0x80;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}
void CanbusClass::SetMasksAndActivateFilters(uint32_t Mask1, uint32_t Mask2, bool RollOver)
{
	if(Mask1)
	{
		SetConfigMode();
		MCP2515_Write(RXB0CTRL, (0<<RXM1) | (0<<RXM0) | (RollOver<<BUKT));	//make filters active on buffer 0
		MCP2515_Write(RXM0SIDH, Mask1>>21);
		MCP2515_Write(RXM0SIDL, ((Mask1>>13) & 0xE0) | ((Mask1>>16) & 0x03));
		MCP2515_Write(RXM0EID8, Mask1>>8);
		MCP2515_Write(RXM0EID0, Mask1);
		SetNormalMode();
	}
	if(Mask2)
	{
		SetConfigMode();
		MCP2515_Write(RXB1CTRL, (0<<RXM1) | (0<<RXM0));	//make filters active on buffer 1
		MCP2515_Write(RXM1SIDH, Mask2>>21);
		MCP2515_Write(RXM1SIDL, ((Mask2>>13) & 0xE0) | ((Mask2>>16) & 0x03));
		MCP2515_Write(RXM1EID8, Mask2>>8);
		MCP2515_Write(RXM1EID0, Mask2);
		SetNormalMode();
	}
}
void CanbusClass::SetFilter(int FilterIndex, uint32_t Filter)
{
	uint8_t FSIDH, FSIDL, FEID8, FEID0;
	setFilterFromIndex(FilterIndex, FSIDH, FSIDL, FEID8, FEID0);

	SetConfigMode();
	MCP2515_Write(FSIDH, Filter>>21);
	MCP2515_Write(FSIDL, ((Filter>>13) & 0xE0) | (1<<EXIDE) | ((Filter>>16) & 0x03));
	MCP2515_Write(FEID8, Filter>>8);
	MCP2515_Write(FEID0, Filter);
	SetNormalMode();
}


void CanbusClass::SetFilterAndMask(int ReceiveBuffer, int FilterIndex, uint32_t Filter, uint32_t Mask)
{
	uint8_t FSIDH, FSIDL, FEID8, FEID0, MSIDH, MSIDL, MEID8, MEID0;
	setFilterFromIndex(FilterIndex, FSIDH, FSIDL, FEID8, FEID0);
	if(ReceiveBuffer == 0)
	{
		MSIDH = RXM0SIDH;
		MSIDL = RXM0SIDL;
		MEID8 = RXM0EID8;
		MEID0 = RXM0EID0;
	}
	else if(ReceiveBuffer == 1)
	{
		MSIDH = RXM1SIDH;
		MSIDL = RXM1SIDL;
		MEID8 = RXM1EID8;
		MEID0 = RXM1EID0;
	}
	else
	{
	}
	SetConfigMode();

	MCP2515_Write(RXB0CTRL, (0<<RXM1) | (0<<RXM0) | (1<<BUKT));	//make filters active on buffer 0
	MCP2515_Write(RXB1CTRL, (0<<RXM1) | (0<<RXM0));	//make filters active on buffer 1
	MCP2515_Write(MSIDH, Mask>>21);
	MCP2515_Write(MSIDL, ((Mask>>13) & 0xE0) | ((Mask>>16) & 0x03));
	MCP2515_Write(MEID8, Mask>>8);
	MCP2515_Write(MEID0, Mask);

	MCP2515_Write(FSIDH, Filter>>21);
	MCP2515_Write(FSIDL, ((Filter>>13) & 0xE0) | (1<<EXIDE) | ((Filter>>16) & 0x03));
	MCP2515_Write(FEID8, Filter>>8);
	MCP2515_Write(FEID0, Filter);

	SetNormalMode();
}

void CanbusClass::setFilterFromIndex(int filterIndex, uint8_t& FSIDH, uint8_t& FSIDL, uint8_t& FEID8, uint8_t& FEID0) {
	switch(filterIndex)
	{
		case 0:
			FSIDH = RXF0SIDH;
			FSIDL = RXF0SIDL;
			FEID8 = RXF0EID8;
			FEID0 = RXF0EID0;
			break;
		case 1:
			FSIDH = RXF1SIDH;
			FSIDL = RXF1SIDL;
			FEID8 = RXF1EID8;
			FEID0 = RXF1EID0;
			break;
		case 2:
			FSIDH = RXF2SIDH;
			FSIDL = RXF2SIDL;
			FEID8 = RXF2EID8;
			FEID0 = RXF2EID0;
			break;
		case 3:
			FSIDH = RXF3SIDH;
			FSIDL = RXF3SIDL;
			FEID8 = RXF3EID8;
			FEID0 = RXF3EID0;
			break;
		case 4:
			FSIDH = RXF4SIDH;
			FSIDL = RXF4SIDL;
			FEID8 = RXF4EID8;
			FEID0 = RXF4EID0;
			break;
		case 5:
			FSIDH = RXF5SIDH;
			FSIDL = RXF5SIDL;
			FEID8 = RXF5EID8;
			FEID0 = RXF5EID0;
			break;
	}
}