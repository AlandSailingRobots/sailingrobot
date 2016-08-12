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
#include "..\SystemServices/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>


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
		uint8_t* buffer = new uint8_t[maxSize];
		bool foundEnd;
		uint16_t index = 0;
		bool slipEscape = false;
		int c = 0;

					std::cout << "FRAME FOR DOGS" << std::endl;
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
			else if((uint8_t)c == SLIP_PACKET_ESCAPE)
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
			delete[] buffer;
			frame.setData(frameData, frameSize);
			return true;
		}

		delete[] buffer;
	}
	
	return false;
}

bool DataLink::foundFrameStart()
{
	//const uint16_t MAX_BYTES_TO_INSPECT = maxFrameSize()*10;

	bool slipEscape = false;
	uint16_t inspected = 0;
	int c = 0;

	// TODO - Jordan: Break out into separate function?

	// Look for start character and break out when found
	while(true)
	{
		// Read enough characters for now, go do something else
		if(inspected >= 1000)
		{
			return false;
		}

		inspected++;
		if(not dataAvailable())
		{
			std::cout << "NOTHING AVAILABLE ;(" << std::endl;
			continue;
		}


		c = readByte();

		// No characters left to read
		if(c == -1)
		{
			return false;
		}

		if((uint8_t)c == SLIP_PACKET_START && not slipEscape)
		{
			break;
		}

		if((uint8_t)c == SLIP_PACKET_ESCAPE)
		{
			slipEscape = true;
		}
		else
		{
			slipEscape = false;
		}

		//std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	return true;

}

