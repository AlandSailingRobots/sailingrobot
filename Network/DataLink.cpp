/****************************************************************************************
 *
 * File:
 * 		DataLink.cpp
 *
 * Purpose:
 *		A data link interacts with the transport layer, it deals with sending and receiving
 *		network frames over the hardware or via a operating system.
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "DataLink.h"
#include "SLIP.h"
#include <cstring>


void DataLink::transmit(const NetworkFrame& frame)
{
	if(m_initialised)
	{
		// Check for valid frame
		if(frame.m_data != NULL && frame.m_size > 0 && frame.m_size <= maxFrameSize())
		{
			writeByte(SLIP_PACKET_START);

			for(unsigned int i = 0; i < frame.m_size; i++)
			{
				if(SLIP::isSpecialCharacter(frame.m_data[i]))
				{
					writeByte(SLIP_PACKET_ESCAPE);
					writeByte(SLIP::getEscapeCharacter(frame.m_data[i]));
				}
				else
				{
					writeByte(frame.m_data[i]);
				}
			}

			writeByte(SLIP_PACKET_END);
		}
	}
}

bool DataLink::receive(NetworkFrame& frame)
{
	if(m_initialised)
	{
		if(not foundFrameStart())
		{
			return false;
		}

		uint16_t maxSize = maxFrameSize();
		uint8_t buffer[maxSize];
		bool foundEnd;
		uint16_t index = 0;
		bool slipEscape = false;
		int c = 0;

		/* Try and find the frame end, our two exit conditions are:
		 * 		- Found the end frame character (SUCCESS)
		 * 		- Gone over the max frame size (FAILURE)
		*/
		while(not foundEnd && index < maxSize)
		{
			c = readByte();

			// No characters left to read
			if(c == -1)
			{
				break;
			}

			// We found the end
			if(not slipEscape && SLIP::isEndCharacter(c))
			{
				foundEnd = true;
			}
			// The last character is escaping a special character
			else if(slipEscape)
			{
				buffer[index] = SLIP::getEscapedCharacter(c);
				index++;
				slipEscape = false;
			}
			// Check if the character is an escape character
			else if(SLIP::isEscapeCharacter(c))
			{
				slipEscape = true;
			}
			// Its a normal character, just store it
			else
			{
				buffer[index] = c;
				index++;
			}
		}

		// Found a full frame
		if(foundEnd)
		{
			uint8_t frameSize = index + 1;
			uint8_t* frameData = new uint8_t[frameSize];
			memcpy(frameData, buffer, frameSize);

			frame.setData(frameData, frameSize);
			return true;
		}
	}

	return false;
}

bool DataLink::foundFrameStart()
{
	const uint16_t MAX_BYTES_TO_INSPECT = maxFrameSize()*2;

	bool slipEscape = false;
	uint16_t inspected = 0;
	int c = 0;

	// TODO - Jordan: Break out into separate function?

	if(not dataAvailable())
	{
		return false;
	}

	// Look for start character and break out when found
	while(true)
	{
		// Read enough characters for now, go do something else
		if(inspected <= MAX_BYTES_TO_INSPECT)
		{
			return false;
		}

		c = readByte();
		inspected++;

		// No characters left to read
		if(c == -1)
		{
			return false;
		}

		if(SLIP::isEscapeCharacter(c))
		{
			slipEscape = true;
		}
		else if(SLIP::isStartCharacter(c) && not slipEscape)
		{
			break;
		}
		else
		{
			slipEscape = false;
		}
	}

	return true;

}

