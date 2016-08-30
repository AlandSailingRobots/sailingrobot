/****************************************************************************************
 *
 * File:
 * 		RazorCompassNode.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "RazorCompassNode.h"
#include "SystemServices/Logger.h"
#include "utility/SysClock.h"
#include "Messages/CompassDataMsg.h"
#include <wiringSerial.h>
#include <cstring>
 #include <termios.h>


#define COMPASS_THREAD_SLEEP		400
#define COMPASS_BAUD_RATE			57600
#define COMPASS_DEFAULT_PORT		"/dev/ttyUSB0"


RazorCompassNode::RazorCompassNode(MessageBus& msgBus)
	:ActiveNode(NodeID::Compass, msgBus), m_FD(-1), m_keepRunning(false)
{

}

RazorCompassNode::~RazorCompassNode()
{
	// TODO Auto-generated destructor stub
	m_keepRunning = false;
}

bool RazorCompassNode::init()
{
	if(m_FD == -1)
	{
		m_FD = serialOpen(COMPASS_DEFAULT_PORT, COMPASS_BAUD_RATE);

		if(m_FD == -1)
		{
			Logger::error("%s Failed to open the serial port", __PRETTY_FUNCTION__);
		}
	}

	return (m_FD != -1);
}

void RazorCompassNode::start()
{
	if(m_FD > -1)
	{
		m_keepRunning = true;
		runThread(RazorThreadFunc);
	}
	else
	{
		Logger::error("%s:%d Cannot start thread, invalid file descriptor", __FILE__, __LINE__);
	}
}

void RazorCompassNode::processMessage(const Message* msg) { }


uint8_t RazorCompassNode::readLine(char* buffer, uint8_t maxLength)
{
	const uint8_t MAX_ATTEMPTS = 25;
	uint8_t charsRead = 0;
	uint8_t attempts = 0;
	bool foundStart = false;

	if(m_FD <= -1)
	{
		Logger::error("%s:%d Invalid file descriptor", __FILE__, __LINE__);
		return 0;
	}

	while( (charsRead < maxLength) && (attempts < MAX_ATTEMPTS) )
	{
		if(serialDataAvail(m_FD) <= 0)
		{
			attempts++;
			SysClock::sleepMS(1);
			continue;
		}

		int c = serialGetchar(m_FD);

		// Indicates some real failure of some kind as we failed to read an available
		// character
		if(c == -1)
		{
			break;
		}

		// Start of the sentence
		if(not foundStart && c == '#')
		{
			foundStart = true;
		}

		if(foundStart)
		{
			buffer[charsRead] = c;
			charsRead++;

			// End of the sentence
			if(c == '\n')
			{
				break;
			}
		}
	}

	if(charsRead < maxLength)
	{
		buffer[charsRead] = '\0';
		charsRead++;
	}
	else
	{
		Logger::warning("No room for NULL terminator");
	}

	return charsRead;
}




bool RazorCompassNode::parseData(char* buffer, float& heading, float& pitch, float& roll)
{
	char* dataStart = 0;
	for(unsigned int i = 0; i < strlen(buffer); i++)
	{
		if(buffer[i] == '=')
		{
			dataStart = buffer + i + 1;
			break;
		}
	}

	if(dataStart != 0)
	{
		int result = sscanf(dataStart, "%f,%f,%f", &heading, &pitch, &roll);
		return result == 3;
	}
	else
	{
		return false;
	}
}

void RazorCompassNode::RazorThreadFunc(void* nodePtr)
{
	RazorCompassNode* node = (RazorCompassNode*)nodePtr;
	char buffer[35];
	uint8_t charsRead = 0;

	float heading = 0;
	float pitch = 0;
	float roll = 0;

	while(node->m_keepRunning)
	{
		SysClock::sleepMS(500);
		tcflush(node->m_FD,TCIOFLUSH);
		SysClock::sleepMS(10);

		charsRead = node->readLine(buffer, 35);

		// Update our version of the data
		if(charsRead > 0)
		{
			if(node->parseData(buffer, heading, pitch, roll))
			{
				Logger::info("Compass Data %f %d %s", heading, (int)(heading + 0.5), buffer);

				MessagePtr msg = std::make_unique<CompassDataMsg>((int)(heading + 0.5), (int)pitch, (int)roll);
				node->m_MsgBus.sendMessage(std::move(msg));
			}
		}

	}
}


