
#ifndef canpgnrecieve_h
#define canpgnrecieve_h

#include <stdint.h>
#include <vector>

class CanPGNRecieve
{
	
public:
	virtual void processPGN(std::vector<uint8_t> Data, uint32_t PGN) = 0;
};

#endif