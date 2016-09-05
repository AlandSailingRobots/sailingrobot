/****************************************************************************************
 *
 * File:
 * 		SerialGPSNode.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "SerialGPSNode.h"
#include "SystemServices/Logger.h"
#include "utility/SysClock.h"
#include "Messages/GPSDataMsg.h"
#include <minmea/minmea.h>
#include <wiringSerial.h>
#include <math.h>


//---------------------------------------------------------------------------------------
// Class Configuration
//---------------------------------------------------------------------------------------


#define GPS_THREAD_SLEEP		100
#define GPS_BAUD_RATE			9600
#define GPS_DEFAULT_PORT		"/dev/ttyAMA0"


//---------------------------------------------------------------------------------------
SerialGPSNode::SerialGPSNode(MessageBus& msgBus)
	:ActiveNode(NodeID::GPS, msgBus), m_FD(-1), m_BaudRate(GPS_BAUD_RATE), m_PortName(GPS_DEFAULT_PORT),
	 m_HasFix(false), m_Lon(NAN), m_Lat(NAN), m_Speed(NAN), m_Course(NAN), m_UnixTime(0), m_keepRunning(false)
{
	// TODO Auto-generated constructor stub

}

//---------------------------------------------------------------------------------------
SerialGPSNode::~SerialGPSNode()
{
	if(m_ThreadPtr != NULL)
	{
		m_keepRunning = false;
		m_ThreadPtr->join();
	}

	delete m_ThreadPtr;
	m_ThreadPtr = NULL;
}

//---------------------------------------------------------------------------------------
bool SerialGPSNode::init()
{
	if(m_BaudRate == 0)
	{
		Logger::error("%s Baud rate is not set!", __PRETTY_FUNCTION__);
	}

	if(m_FD == -1)
	{
		m_FD = serialOpen(m_PortName.c_str(), m_BaudRate);

		if(m_FD == -1)
		{
			Logger::error("%s Failed to open the serial port", __PRETTY_FUNCTION__);
		}
	}

	return (m_FD != -1);
}

//---------------------------------------------------------------------------------------
void SerialGPSNode::processMessage(const Message* message)
{
	// Not expecting anything
}

//---------------------------------------------------------------------------------------
void SerialGPSNode::start()
{
	if(m_FD != -1)
	{
		m_keepRunning = true;
		runThread(GPSThreadFunc);
	}
	else
	{
		Logger::error("%s:%d Failed to start thread, no valid file descriptor (init may of failed or wasn't called", __FILE__, __LINE__);
	}
}

//---------------------------------------------------------------------------------------
uint8_t SerialGPSNode::readNMEALine(char* nmeaBuffer, uint8_t maxLength)
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
		if(not foundStart && c == '$')
		{
			foundStart = true;
		}

		if(foundStart)
		{
			nmeaBuffer[charsRead] = c;
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
		nmeaBuffer[charsRead] = '\0';
		charsRead++;
	}
	else
	{
		Logger::warning("No room for NULL terminator");
	}

	return charsRead;
}

//---------------------------------------------------------------------------------------
void SerialGPSNode::processNMEA(char* nmeaBuffer)
{
	static bool setSysClock = false;

	switch (minmea_sentence_id(nmeaBuffer, false))
	{
		case MINMEA_SENTENCE_RMC:
		{
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, nmeaBuffer))
			{
				m_Lon = minmea_tocoord(&frame.longitude);
				m_Lat = minmea_tocoord(&frame.latitude);
				m_Speed = minmea_tofloat(&frame.speed);
				m_Course = minmea_tofloat(&frame.course);

				struct timespec ts;
				m_UnixTime = minmea_gettime(&ts, &frame.date, &frame.time);

				if(setSysClock == false && m_HasFix)
				{
					setSysClock = true;
					SysClock::setTime(m_UnixTime);
					Logger::info("GPS set SysClock time");
				}
			}
		} break;

		case MINMEA_SENTENCE_GGA:
		{
			struct minmea_sentence_gga frame;
			if (minmea_parse_gga(&frame, nmeaBuffer))
			{
				m_HasFix = (frame.fix_quality > 0);
			}
		} break;

		default:
			break;
	}
}

//---------------------------------------------------------------------------------------
void SerialGPSNode::GPSThreadFunc(void* nodePtr)
{
	SerialGPSNode* node = (SerialGPSNode*)nodePtr;
	char nmeaBuffer[MINMEA_MAX_LENGTH];
	uint8_t charsRead = 0;

	while(node->m_keepRunning)
	{
		if(not node->hasFix())
		{
			SysClock::sleepMS(GPS_THREAD_SLEEP / 2);
		}
		else
		{
			SysClock::sleepMS(GPS_THREAD_SLEEP);
			Logger::info("Has GPS Fix");
		}

		charsRead = node->readNMEALine(nmeaBuffer, MINMEA_MAX_LENGTH);

		// Update our version of the data
		if(charsRead > 0)
		{
			node->processNMEA(nmeaBuffer);
		}


		MessagePtr msg = std::make_unique<GPSDataMsg>(node->m_HasFix, true, node->m_Lat, node->m_Lon, node->m_UnixTime, node->m_Speed, node->m_Course, 0, GPSMode::LatLonOk);
		node->m_MsgBus.sendMessage(std::move(msg));
	}
}


