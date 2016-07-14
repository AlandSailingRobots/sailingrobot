/****************************************************************************************
 *
 * File:
 * 		CV7Node.cpp
 *
 * Purpose:
 *		The CV7 node provides wind data to the system using the CV7 wind sensor.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "CV7Node.h"
#include "logger/Logger.h"
#include "dbhandler/DBHandler.h"
#include "Messages/WindDataMsg.h"

// For std::this_thread
#include <chrono>
#include <thread>

#include "wiringSerial.h"


#define WIND_SENSOR_SLEEP_MS	100


CV7Node::CV7Node(MessageBus& msgBus, std::string portName, unsigned int baudRate)
	:ActiveNode(NodeID::WindSensor, msgBus), m_Initialised(false), m_fd(-1), m_PortName(portName), m_BaudRate(baudRate)
{

}

bool CV7Node::init()
{
	bool success = true;
	m_fd = serialOpen(m_PortName.c_str(), m_BaudRate);

	if(m_fd < 0)
	{
		success = false;
	}

	m_Initialised = true;

	return success;
}

void CV7Node::start()
{
	if(m_Initialised)
	{
		runThread(WindSensorThread);
	}
	else
	{
		Logger::error("%s Cannot start wind sensor thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
}

void CV7Node::processMessage(const Message* message)
{

}

void CV7Node::WindSensorThread(void* nodePtr)
{
	CV7Node* node = (CV7Node*)(nodePtr);

	const int DATA_BUFFER_SIZE = 30;
	const int NON_BREAKING_SPACE = 255;
	const int BUFF_SIZE = 256;
	char buffer[BUFF_SIZE];

	std::vector<float> windDirData(DATA_BUFFER_SIZE);
	std::vector<float> windSpeedData(DATA_BUFFER_SIZE);
	std::vector<float> windTempData(DATA_BUFFER_SIZE);
	unsigned int dataIndex = 0;

	Logger::info("Wind sensor thread started");

	while(true)
	{
		// Controls how often we pump out messages
		std::this_thread::sleep_for(std::chrono::milliseconds(WIND_SENSOR_SLEEP_MS));

		// Extract data from the CV7 serial port
		int index = 0;
		while(index < BUFF_SIZE) {

			buffer[index] = serialGetchar(node->m_fd);
			fflush(stdout);

			if(NON_BREAKING_SPACE == (int)buffer[index])
			{
				Logger::warning("Wind sensor serial timeout!");
				index = 0;
				break;
			}
			index++;
		}

		// Parse the data and send out messages
		if(index > 0 )
		{
			float windDir = 0;
			float windSpeed = 0;
			float windTemp = 0;

			if(CV7Node::parseString(buffer, windDir, windSpeed, windTemp))
			{
				if(windDirData.size() < DATA_BUFFER_SIZE)
				{
					windDirData.push_back(windDir);
					windSpeedData.push_back(windSpeed);
					windTempData.push_back(windTemp);
				}
				else
				{
					if(dataIndex == DATA_BUFFER_SIZE)
					{
						dataIndex = 0;
					}

					windDirData[dataIndex] = windDir;
					windSpeedData[dataIndex] = windSpeed;
					windTempData[dataIndex] = windTemp;
					dataIndex++;
				}

				// Send a wind sensor message out
				WindDataMsg* windData = new WindDataMsg()
			}
		}
	}
}

static bool CV7Node::parseString(const char* buffer, float& windDir, float& windSpeed, float& windTemp)
{
	const int IIMWV = 0;
	const int WIXDR = 1;
	bool updated[] = { false, false };
	char * writalbeBuff;
	writalbeBuff = const_cast<char *>(buffer);
	char* split = strtok(writalbeBuff, "$,");

	while (split != NULL)
	{
		if (strcmp(split, "IIMWV") == 0)
		{
			split = strtok(NULL, "$,");
			windDir = atof(split);
			split = strtok(NULL, "$,");
			split = strtok(NULL, "$,");
			windSpeed = atof(split);
			updated[IIMWV] = true;
		} else if (strcmp(split, "WIXDR") == 0)
		{
			split = strtok(NULL, "$,");
			split = strtok(NULL, "$,");
			windTemp = atof(split);
			updated[WIXDR] = true;
		}

		if (updated[IIMWV] && updated[WIXDR])
		{
			break;
		}
		split = strtok(NULL, "$,");
	}
	if (updated[IIMWV] == false || updated[WIXDR] == false ) {
		return false;
	}

	return true;
}
