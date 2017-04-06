#ifndef	MCP2515_H
#define	MCP2515_H

#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "mcp2515_defs.h"
#include "N2kMsg.h"

// ----------------------------------------------------------------------------

void MCP2515_SendByte(uint8_t Data);

uint8_t MCP2515_Read(uint8_t Address);

void MCP2515_Write(uint8_t Address, uint8_t Data);

uint8_t MCP2515_ReadStatus();

uint8_t MCP2515_RXStatus();

void MCP2515_BitModify(uint8_t Address, uint8_t Mask, uint8_t Data);

void MCP2515_RequestToSend(uint8_t Buffers);

uint8_t MCP2515_ReadRXBuffer(uint8_t Address);

void MCP2515_LoadTXBuffer(uint8_t Address, uint8_t Data);

bool MCP2515_Init();

uint8_t MCP2515_CheckMessage();

uint8_t MCP2515_CheckFreeBuffer();

bool MCP2515_GetMessage(CanMsg *message, uint8_t MsgAddress);

uint8_t MCP2515_SendMessage(CanMsg *message, uint8_t MsgAddress);

void MCP2515_OutputInfo();

#endif	// MCP2515_H
