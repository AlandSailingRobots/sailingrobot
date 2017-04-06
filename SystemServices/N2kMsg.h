#pragma once

#include <stdint.h>
#include <vector>

struct CanMsg
{
	uint32_t id;
	struct
	{
		//uint8_t rtr;		//Always zero in J1939
		uint8_t ide;
		uint8_t length;
	} header;
	uint8_t data[8];
};

struct N2kMsg
{
 	uint32_t PGN;
 	uint8_t Priority;
 	uint8_t Source;
 	uint8_t Destination;
 	int DataLen;
  std::vector<uint8_t> Data;
};

void IdToN2kMsg(N2kMsg &NMsg, uint32_t &id);