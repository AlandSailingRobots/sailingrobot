#include <stdint.h>

#include <unistd.h>


#include "mcp2515.h"
#include "mcp2515_defs.h"
#include <SPI.h>

void inline ReadWriteSPI(uint8_t *ReadWrite, uint8_t Bytes)
{
    digitalWrite(m_chipSelectPin, LOW);
    SPI.transfer(ReadWrite, Bytes);
    digitalWrite(m_chipSelectPin, HIGH);
}

void MCP2515_SendByte(uint8_t Data)
{
    uint8_t ReadWrite = Data;
    ReadWriteSPI(ReadWrite, 1);
}

uint8_t MCP2515_Read(uint8_t Address)
{
    uint8_t ReadWrite[3];
    ReadWrite[0] = SPI_READ;
    ReadWrite[1] = Address;
    ReadWrite[2] = 0x00;		//data is written here
    ReadWriteSPI(ReadWrite, 3);
    return ReadWrite[2];
}

void MCP2515_Write(uint8_t Address, uint8_t Data)
{
    uint8_t ReadWrite[3];
    ReadWrite[0] = SPI_WRITE;
    ReadWrite[1] = Address;
    ReadWrite[2] = Data;
    ReadWriteSPI(ReadWrite, 3);
}

uint8_t MCP2515_ReadStatus()
{
    uint8_t ReadWrite[2];
    ReadWrite[0] = SPI_READ_STATUS;
    ReadWrite[1] = 0x00;		//data is written here
    ReadWriteSPI(ReadWrite, 2);
    return ReadWrite[1];
}

uint8_t MCP2515_RXStatus()
{
    uint8_t ReadWrite[2];
    ReadWrite[0] = SPI_RX_STATUS;
    ReadWrite[1] = 0x00;		//data is written here
    ReadWriteSPI(ReadWrite, 2);
    return ReadWrite[1];
}

void MCP2515_BitModify(uint8_t Address, uint8_t Mask, uint8_t Data)
{
    uint8_t ReadWrite[4];
    ReadWrite[0] = SPI_BIT_MODIFY;
    ReadWrite[1] = Address;
    ReadWrite[2] = Mask;
    ReadWrite[3] = Data;
    ReadWriteSPI(ReadWrite, 4);
}

bool MCP2515_Init(int chipSelectPin)
{
    m_chipSelectPin = chipSelectPin;
    // reset MCP2515 by software reset.
    // After this it is in configuration mode.
    MCP2515_SendByte(SPI_RESET);
    delay(10);

    // load CNF1..3 Register
    uint8_t ReadWrite[6];
    ReadWrite[0] = SPI_WRITE;
    ReadWrite[1] = CNF3;

    ReadWrite[2] = 0x05;					// Bitrate 250 kbps at 16 MHz		//CNF3
    ReadWrite[3] = 0xf1;										//CNF2
    ReadWrite[4] = 0x41;										//CNF1

    ReadWrite[5] = (1<<RX1IE)|(1<<RX0IE);		// activate interrupts
    //wiringPiSPIDataRW(CHANNEL, ReadWrite, 6);
    ReadWriteSPI(ReadWrite, 6);
    // test if we could read back the value => is the chip accessible?

    if(MCP2515_Read(CNF1) != 0x41)
    {
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
            return false;
        }
    }

    uint8_t ReadWrite[14] = {0};			//Address + 2 std_id bytes + 2 ext_id bytes + data length + 8 data bytes
    ReadWrite[0] = MsgAddr;					//Read RX Buffer automatically clears the receive flag

    ReadWriteSPI(ReadWrite, 14);
    message->header.ide = (ReadWrite[2]>>IDE) & 0x01;


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

    for (uint8_t i = 0; i < Length; ++i)
    {
        ReadWrite[6+i] = message->data[i];	//data byte TXBnDm
    }


    ReadWriteSPI(ReadWrite, 6+Length);

    delay(10);

    // send message
    Address &= 0x0f;
    Address = (Address == 0) ? 1 : Address;
    MCP2515_SendByte(SPI_RTS | Address);	//request to send
    return Address;
}

void MCP2515_OutputInfo()
{
    uint8_t CTRL = MCP2515_Read(CANCTRL);

    uint8_t T0C = MCP2515_Read(TXB0CTRL);
    uint8_t T1C = MCP2515_Read(TXB1CTRL);
    uint8_t T2C = MCP2515_Read(TXB2CTRL);

    //uint8_t INTE = MCP2515_Read(CANINTE);
    uint8_t INTF = MCP2515_Read(CANINTF);

    uint8_t EF = MCP2515_Read(EFLG);

    uint8_t TXRTS = MCP2515_Read(TXRTSCTRL);
}
