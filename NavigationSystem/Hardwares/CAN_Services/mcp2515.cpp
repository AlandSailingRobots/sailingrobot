#include <stdint.h>
#include <iostream>
#include <unistd.h>

#include "can_rpi_defs.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>

void MCP2515_SendByte(uint8_t Data)
{
	uint8_t ReadWrite = Data;
	wiringPiSPIDataRW(CHANNEL, &ReadWrite, 1);
}

uint8_t MCP2515_Read(uint8_t Address)
{
	uint8_t ReadWrite[3];
	ReadWrite[0] = SPI_READ;
	ReadWrite[1] = Address;
	ReadWrite[2] = 0x00;		//data is written here
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 3) ;
	return ReadWrite[2];
}

void MCP2515_Write(uint8_t Address, uint8_t Data)
{
	uint8_t ReadWrite[3];
	ReadWrite[0] = SPI_WRITE;
	ReadWrite[1] = Address;
	ReadWrite[2] = Data;
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 3);
}

uint8_t MCP2515_ReadStatus()
{
	uint8_t ReadWrite[2];
	ReadWrite[0] = SPI_READ_STATUS;
	ReadWrite[1] = 0x00;		//data is written here
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 2);
	return ReadWrite[1];
}

uint8_t MCP2515_RXStatus()
{
	uint8_t ReadWrite[2];
	ReadWrite[0] = SPI_RX_STATUS;
	ReadWrite[1] = 0x00;		//data is written here
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 2);
	return ReadWrite[1];
}

void MCP2515_BitModify(uint8_t Address, uint8_t Mask, uint8_t Data)
{
	uint8_t ReadWrite[4];
	ReadWrite[0] = SPI_BIT_MODIFY;
	ReadWrite[1] = Address;
	ReadWrite[2] = Mask;
	ReadWrite[3] = Data;
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 4);
}

bool MCP2515_Init()
{
	// reset MCP2515 by software reset.
	// After this it is in configuration mode.
	MCP2515_SendByte(SPI_RESET);
#ifdef VERBOSE
	std::cout << "waiting for CAN bus to restart\n";	// wait a little bit until the MCP2515 has restarted
#endif
	usleep(10);

	// load CNF1..3 Register
	uint8_t ReadWrite[6];
	ReadWrite[0] = SPI_WRITE;
	ReadWrite[1] = CNF3;

	ReadWrite[2] = 0x05;					// Bitrate 250 kbps at 16 MHz		//CNF3
	ReadWrite[3] = 0xf1;										//CNF2
	ReadWrite[4] = 0x41;										//CNF1

	ReadWrite[5] = (1<<RX1IE)|(1<<RX0IE);		// activate interrupts
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 6);

	// test if we could read back the value => is the chip accessible?

	if(MCP2515_Read(CNF1) != 0x41)
	{
#ifdef VERBOSE
		std::cout << "Chip not accessible\n";
#endif
		return false;
	}

	// deactivate the RXnBF Pins (High Impedance State)
	MCP2515_Write(BFPCTRL, 0);

	// set TXnRTS as inputs
	MCP2515_Write(TXRTSCTRL, 0);

	// turn off filters => receive any message
	MCP2515_Write(RXB0CTRL, (1<<RXM1) | (1<<RXM0) | (1<<BUKT));			//no filters, rollover from buffer 0 to 1
//	MCP2515_Write(RXB0CTRL, (1<<RXM1) | (1<<RXM0));
	MCP2515_Write(RXB1CTRL, (1<<RXM1) | (1<<RXM0));

	// reset device to normal mode
	MCP2515_Write(CANCTRL, 0);

	return true;
}

bool PickFirst = true;
uint8_t MCP2515_CheckMessage()		// check if there are any new messages waiting
{
	uint8_t MsgAddr;
	uint8_t RXStatus = MCP2515_RXStatus();	// read RX Status
	if((RXStatus & (1 << 6)) || (RXStatus & (1 << 7)))		//if one
	{
		if((RXStatus & (1 << 6)) && (RXStatus & (1 << 7)))	//if both
		{
			if(PickFirst)						//alternate
			{
				MsgAddr = SPI_READ_RX;			//RXB0SIDH STANDARD IDENTIFIER HIGH 0x61
				PickFirst = false;
			}
			else
			{
				MsgAddr = SPI_READ_RX | 0x04;	//RXB1SIDH STANDARD IDENTIFIER HIGH 0x71
				PickFirst = true;
			}
		}
		else									//pick the one having a message
		{
			PickFirst = true;
			if(RXStatus & (1 << 6))
			{
				MsgAddr = SPI_READ_RX;			//RXB0SIDH STANDARD IDENTIFIER HIGH 0x61
			}
			else
			{
				MsgAddr = SPI_READ_RX | 0x04;	//RXB1SIDH STANDARD IDENTIFIER HIGH 0x71
			}
		}
	}
	else
	{
		//std::cout << "No messages avaliable\n";
		return 0;
	}
	return MsgAddr;
}

bool MCP2515_GetMessage(CanMsg *message, uint8_t MsgAddress)
{
	uint8_t MsgAddr;
	if(MsgAddress)						//if an address was supplied
	{
		MsgAddr = MsgAddress;
	}
	else
	{
		MsgAddr = MCP2515_CheckMessage();
		if(MsgAddr == 0)
		{
			//std::cout << "cannot get message\n";
			return false;
		}
	}

	uint8_t ReadWrite[14] = {0};			//Address + 2 std_id bytes + 2 ext_id bytes + data length + 8 data bytes
	ReadWrite[0] = MsgAddr;					//Read RX Buffer automatically clears the receive flag

	wiringPiSPIDataRW(CHANNEL, ReadWrite, 14);
	message->header.ide = (ReadWrite[2]>>IDE) & 0x01;
	//if(message->header.ide)				//Always zero in J1939
	//{
	//	message->header.rtr = (ReadWrite[5]>>RTR) & 0x01;
	//}
	//else
	//{
	//	message->header.rtr = (ReadWrite[2]>>SRR) & 0x01;
	//}

	// read id
	message->id = 0;
	message->id  = (uint32_t) ReadWrite[1] << 3;	//standard id high bits	<10:3>		TXBnSIDH
	message->id |=            ReadWrite[2] >> 5;	//standard id low bits	<2:0>		TXBnSIDL
	if(message->header.ide)
	{
		message->id = (message->id << 18);						//shift the standard id up
		message->id |= (uint32_t)(ReadWrite[2] & 0x03) << 16;	//mask bit 0-1, extended id high bits <17:16>	TXBnSIDL
		message->id |= (uint32_t)ReadWrite[3] << 8;				//extended id <15:8>	TXBnEID8
		message->id |= (uint32_t)ReadWrite[4];					//extended id <7:0>		TXBnEID0
	}
	uint8_t Length = ReadWrite[5] & 0x0f;
	message->header.length = Length;
	for (uint8_t i=0; i < Length; ++i)
	{
		message->data[i] = ReadWrite[6+i];
	}

	//return (RXStatus & 0x07) + 1;	//return which filter it matched
	return true;
}

uint8_t MCP2515_CheckFreeBuffer()		// check if there is a free buffer to send messages
{
	uint8_t Status = MCP2515_ReadStatus();

	if(!(Status & (1 << 2)))		//check the TXREQ flags to find an empty buffer
	{
		return SPI_WRITE_TX;		// |0x00;
	}
	else if(!(Status & (1 << 4)))
	{
		return (SPI_WRITE_TX|0x02);
	}
	else if(!(Status & (1 << 6)))
	{
		return (SPI_WRITE_TX|0x04);
	}
	else
	{
#ifdef VERBOSE
		std::cout << "no free buffers\n";
#endif
		return 0;
	}
}

uint8_t MCP2515_SendMessage(CanMsg *message, uint8_t MsgAddress)			//returns which register it used
{
	uint8_t Address = SPI_WRITE_TX;
	if(MsgAddress)						//if an address was supplied
	{
		Address = MsgAddress;
	}
	else
	{
		Address = MCP2515_CheckFreeBuffer();
		if (Address == 0)
		{
#ifdef VERBOSE
			std::cout << "cannot send message\n";
#endif
			return 0;
		}
	}

	uint8_t ReadWrite[14] = {0};				//Address + 2 std_id bytes + 2 ext_id bytes + data length + 8 data bytes
	ReadWrite[0] = Address;
	if(message->header.ide)
	{
		ReadWrite[1] = (message->id>>18) >> 3;					//standard identifier high bits	<10:3> 		TXBnSIDH
		ReadWrite[2] = (message->id>>18) << 5;					//standard identifier low bits 	<2:0> 7-5	TXBnSIDL

		ReadWrite[2] |= ((message->id >> 16)& 0x03) | (1<<EXIDE);	//extended identifier high bits <17:16> 1-0 + IDE bit 	TXBnSIDL
		ReadWrite[3] = message->id >> 8;							//extended id high bits			<15:8>	TXBnEID8
		ReadWrite[4] = message->id;									//extended id low bits 			<7:0>	TXBnEID0
	}
	else
	{
		ReadWrite[1] = message->id >> 3;					//standard identifier high bits	0-7 		TXBnSIDH
		ReadWrite[2] = message->id << 5;					//standard identifier low bits 	5-7 		TXBnSIDL
		ReadWrite[3] = 0;
		ReadWrite[4] = 0;
	}
	uint8_t Length = message->header.length;
	ReadWrite[5] = Length;								//data length code  TXBnDLC

	//if (message->header.rtr)			//if remote transmission request, always zero in J1939
	//{
	//	// a rtr-frame has a length, but contains no data
	//	if(message->header.ide)
	//	{
	//		ReadWrite[5] |= (1<<RTR);		//data lenght code  TXBnDLC
	//	}
	//	else
	//	{
	//		ReadWrite[2] |= (1<<SRR);		//data lenght code  TXBnDLC
	//	}
	//	Length = 0;
	//}
	//else
	{
		for (uint8_t i = 0; i < Length; ++i)
		{
			ReadWrite[6+i] = message->data[i];	//data byte TXBnDm
		}
	}
#ifdef VERBOSE
	std::cout << "sending message\n";
#endif
	wiringPiSPIDataRW(CHANNEL, ReadWrite, 6+Length);

	usleep(10);

	// send message
	Address &= 0x0f;
	Address = (Address == 0) ? 1 : Address;
	MCP2515_SendByte(SPI_RTS | Address);	//request to send
	return Address;
}

void MCP2515_OutputInfo()
{
	uint8_t CTRL = MCP2515_Read(CANCTRL);
	std::cout << "CANCTRL register\n";
	std::cout << "REQOP\tABAT\tOSM\tCLKEN\tCLKPRE\n";
	std::cout << ((CTRL>>7)&1)<<((CTRL>>6)&1)<<((CTRL>>5)&1) <<'\t'<< ((CTRL>>4)&1) <<'\t'<< ((CTRL>>3)&1) <<'\t'<< ((CTRL>>2)&1) <<'\t'<< ((CTRL>>1)&1)<<(CTRL&1) <<std::endl;


	uint8_t T0C = MCP2515_Read(TXB0CTRL);
	uint8_t T1C = MCP2515_Read(TXB1CTRL);
	uint8_t T2C = MCP2515_Read(TXB2CTRL);
	std::cout << "TXBnCTRL registers\n";
	std::cout << "n\tABTF\tMLOA\tTXERR\tTXREQ\tTXP\n";
	std::cout << "0\t" << ((T0C >> 6) & 1) << '\t' <<((T0C >> 5) & 1) << '\t' << ((T0C >> 4) & 1) << '\t' << ((T0C >> 3) & 1) << '\t' << (T0C & 0x03) << std::endl;
	std::cout << "1\t" << ((T1C >> 6) & 1) << '\t' <<((T1C >> 5) & 1) << '\t' << ((T1C >> 4) & 1) << '\t' << ((T1C >> 3) & 1) << '\t' << (T1C & 0x03) << std::endl;
	std::cout << "2\t" << ((T2C >> 6) & 1) << '\t' <<((T2C >> 5) & 1) << '\t' << ((T2C >> 4) & 1) << '\t' << ((T2C >> 3) & 1) << '\t' << (T2C & 0x03) << std::endl;

	//uint8_t INTE = MCP2515_Read(CANINTE);
	uint8_t INTF = MCP2515_Read(CANINTF);
	std::cout << "Interrupts:\tMERR\tWAK\tERR\tTX2\tTX1\tTX0\tRX1\tRX0\n";
	//std::cout << "Enabled:   \t" << ((INTE>>7)&1) <<'\t'<< ((INTE>>6)&1) <<'\t'<< ((INTE>>5)&1) <<'\t';
	//std::cout << ((INTE>>4)&1) <<'\t'<< ((INTE>>3)&1) <<'\t'<< ((INTE>>2)&1) <<'\t'<< ((INTE>>1)&1) <<'\t'<< (INTE&1) <<std::endl;
	std::cout << "Flagged:   \t" << ((INTF>>7)&1) <<'\t'<< ((INTF>>6)&1) <<'\t'<< ((INTF>>5)&1) <<'\t';
	std::cout << ((INTF>>4)&1) <<'\t'<< ((INTF>>3)&1) <<'\t'<< ((INTF>>2)&1) <<'\t'<< ((INTF>>1)&1) <<'\t'<< (INTF&1) <<std::endl;

	uint8_t EF = MCP2515_Read(EFLG);
	std::cout << "Error flags:\n";
	std::cout << "RX1OVR\tRX0OVR\tTXBO\tTXEP\tRXEP\tTXWAR\tRXWAR\tEWARN\n";
	std::cout << ((EF>>7)&1) <<'\t'<< ((EF>>6)&1) <<'\t'<< ((EF>>5)&1) <<'\t'<< ((EF>>4)&1) <<'\t'<< ((EF>>3)&1) <<'\t'<< ((EF>>2)&1) <<'\t'<< ((EF>>1)&1) <<'\t'<< (EF&1) <<std::endl;

	uint8_t TXRTS = MCP2515_Read(TXRTSCTRL);
	std::cout << "Pin control and status\n";
	std::cout << "B2RTS\tB1RTS\tB0RTS\tB2RTSM\tB1RTSM\tB0RTSM\n";
	std::cout << ((TXRTS>>5)&1) <<'\t'<< ((TXRTS>>4)&1) <<'\t'<< ((TXRTS>>3)&1) <<'\t'<< ((CTRL>>2)&1) <<'\t'<< ((TXRTS>>1)&1) <<'\t'<< (TXRTS&1) <<std::endl;
}
