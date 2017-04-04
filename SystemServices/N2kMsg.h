#pragma once

struct N2kMsg
{
 	uint32_t PGN;
 	uint8_t Priority;
 	uint8_t Source;
 	uint8_t Destination;
 	int DataLen;
  std::vector<uint8_t> Data;
};
