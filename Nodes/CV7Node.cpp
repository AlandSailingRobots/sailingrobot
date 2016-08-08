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
#include "SystemServices/Logger.h"
#include "dbhandler/DBHandler.h"
#include "Messages/WindDataMsg.h"
#include "utility/Utility.h"
#include <cstring>

// For std::this_thread
#include <chrono>
#include <thread>

#include "wiringSerial.h"


#define WIND_SENSOR_SLEEP_MS	100
#define DATA_OUT_OF_RANGE		-2000.


CV7Node::CV7Node(MessageBus& msgBus, std::string portName, unsigned int baudRate)
	:ActiveNode(NodeID::WindSensor, msgBus), m_Initialised(false), m_fd(-1), m_PortName(portName), m_BaudRate(baudRate),
	 m_MeanWindDir(DATA_OUT_OF_RANGE), m_MeanWindSpeed(DATA_OUT_OF_RANGE), m_MeanWindTemp(DATA_OUT_OF_RANGE)
{

}

bool CV7Node::init()
{
	m_fd = serialOpen(m_PortName.c_str(), m_BaudRate);

	if(m_fd != 0)
	{
		m_Initialised = true;
	}

	return m_Initialised;
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
	if(message->messageType() == MessageType::DataRequest)
	{
		// On system startup we won't have any valid data, so don't send any
		if(m_MeanWindDir != DATA_OUT_OF_RANGE ||  m_MeanWindTemp != DATA_OUT_OF_RANGE || m_MeanWindSpeed != DATA_OUT_OF_RANGE)
		{
			MessagePtr windData = std::make_unique<WindDataMsg>(message->sourceID(), this->nodeID(), m_MeanWindDir, m_MeanWindTemp, m_MeanWindSpeed);
			m_MsgBus.sendMessage(std::move(windData));
		}
	}
}

void CV7Node::WindSensorThread(void* nodePtr)
{
	CV7Node* node = (CV7Node*)(nodePtr);

	const int DATA_BUFFER_SIZE = 1;
	const int NON_BREAKING_SPACE = 255;
	const int BUFF_SIZE = 60;
	const short MAX_NO_DATA_ERROR_COUNT = 100; // 10 seconds of no data
	char buffer[BUFF_SIZE];

	std::vector<float> windDirData(DATA_BUFFER_SIZE);
	std::vector<float> windSpeedData(DATA_BUFFER_SIZE);
	std::vector<float> windTempData(DATA_BUFFER_SIZE);
	unsigned int dataIndex = 0;
	unsigned short noDataCount = 0;
	Logger::info("Wind sensor thread started");

	while(true)
	{
		if(noDataCount >= MAX_NO_DATA_ERROR_COUNT)
		{
			Logger::error("%s No data on the CV7 serial line!", __PRETTY_FUNCTION__);
			noDataCount = 0;
		}

		// Controls how often we pump out messages
		std::this_thread::sleep_for(std::chrono::milliseconds(WIND_SENSOR_SLEEP_MS));

		// Extract data from the CV7 serial port
		int index = 0;
		for(int i = 0; i < BUFF_SIZE; i++) {

			int data = serialGetchar(node->m_fd);
			if(data != -1)
			{
				buffer[index] = data;

				if(NON_BREAKING_SPACE == (int)buffer[index])
				{
					Logger::warning("Wind sensor serial timeout!");
					index = 0;
					break;
				}
				index++;
			}
			else
			{
				Logger::error("%s No data on the CV7 serial line!", __PRETTY_FUNCTION__);
			}
		}

		// Parse the data and send out messages
		if(index > 0 )
		{
			float windDir = 0;
			float windSpeed = 0;
			float windTemp = 0;

			if(node->parseString(buffer, windDir, windSpeed, windTemp))
			{
				noDataCount = 0;
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

				node->m_MeanWindDir = Utility::meanOfAngles(windDirData);
				node->m_MeanWindTemp = Utility::mean(windTempData);
				node->m_MeanWindSpeed = Utility::mean(windSpeedData);

				// Send a wind sensor message out

				MessagePtr windData = std::make_unique<WindDataMsg>(node->m_MeanWindDir, node->m_MeanWindSpeed, node->m_MeanWindTemp);
				node->m_MsgBus.sendMessage(std::move(windData));

			}
		}
		else
		{
			noDataCount++;
		}
	}
}

bool CV7Node::parseString(const char* buffer, float& windDir, float& windSpeed, float& windTemp) const
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
