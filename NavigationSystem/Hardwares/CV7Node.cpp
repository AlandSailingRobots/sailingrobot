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
#include "../SystemServices/Logger.h"
#include "../Database/DBHandler.h"
#include "../Messages/WindDataMsg.h"
#include "../Math/Utility.h"
#include <cstring>

#include <stdio.h>
#include <regex>

#include <unistd.h>
#include <cerrno>

// For std::this_thread
#include <chrono>
#include <thread>

#include "wiringSerial.h"


#define WIND_SENSOR_SLEEP_MS	100
#define DATA_OUT_OF_RANGE		-2000.


CV7Node::CV7Node(MessageBus& msgBus, DBHandler& dbhandler)
	:ActiveNode(NodeID::WindSensor, msgBus), m_Initialised(false), m_fd(-1), m_PortName("/dev/ttyS0"), m_BaudRate(4800),
	 m_MeanWindDir(DATA_OUT_OF_RANGE), m_MeanWindSpeed(DATA_OUT_OF_RANGE), m_MeanWindTemp(DATA_OUT_OF_RANGE), m_LoopTime(0.5),
	 m_db(dbhandler)
{
	msgBus.registerNode(*this, MessageType::DataRequest);
	msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
	m_BaudRate = m_db.retrieveCellAsInt("config_wind_sensor","1","baud_rate");
	m_PortName = m_db.retrieveCell("config_wind_sensor","1","port");
}

CV7Node::~CV7Node()
{
	if (m_fd!=-1)
	  close(m_fd);
}

bool CV7Node::init()
{
	updateConfigsFromDB();
  m_fd = serialOpen(m_PortName.c_str(),m_BaudRate);


	if(m_fd!=-1)
	{
		m_Initialised = true;
	}
	else
	{
		Logger::error("%s Cannot open %s as windsensor: %s", __PRETTY_FUNCTION__,m_PortName.c_str(),strerror(errno));
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

void CV7Node::updateConfigsFromDB()
{
	m_LoopTime = m_db.retrieveCellAsDouble("config_wind_sensor","1","loop_time");
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
	else if(message->messageType() == MessageType::ServerConfigsReceived)
	{
		updateConfigsFromDB();
	}
}

void CV7Node::WindSensorThread(ActiveNode* nodePtr)
{
	CV7Node* node = dynamic_cast<CV7Node*> (nodePtr);

	const int DATA_BUFFER_SIZE = 1;
	const int BUFF_SIZE = 255;
	char buffer[BUFF_SIZE];


	std::string buffer_to_parse;
	std::vector<float> windDirData(DATA_BUFFER_SIZE);
	std::vector<float> windSpeedData(DATA_BUFFER_SIZE);
	std::vector<float> windTempData(DATA_BUFFER_SIZE);
	unsigned int dataIndex = 0;
	fd_set fd_read_set;
	struct timeval wait_time;


	Logger::info("Wind sensor thread  ");
	buffer_to_parse.reserve(255);

	float windDir = 0;
	float windSpeed = 0;
	float windTemp = 0;

	while(true)
	{
    int bytes = 0;
		wait_time.tv_sec = 10;
		wait_time.tv_usec = 0;
		bzero(buffer, BUFF_SIZE);

    FD_ZERO(&fd_read_set);
    FD_SET(node->m_fd, &fd_read_set);

    /* Wait for data to come or timeout*/
    select(node->m_fd+1, &fd_read_set, NULL, NULL, &wait_time);

    if (FD_ISSET(node->m_fd, &fd_read_set))
    {
        bytes = read(node->m_fd, buffer, BUFF_SIZE);
    }
		else
		{
			Logger::error("%s No data on the CV7 serial line!", __PRETTY_FUNCTION__);
		}


		// Parse the data and send out messages
		if(bytes > 0 )
		{
			buffer_to_parse+=buffer;

      //Logger::info("buffer windsensor %s",buffer_to_parse.c_str());
			if(node->parseString(buffer_to_parse, windDir, windSpeed, windTemp))
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

				node->m_MeanWindDir = Utility::meanOfAngles(windDirData);
				node->m_MeanWindTemp = Utility::mean(windTempData);
				node->m_MeanWindSpeed = Utility::mean(windSpeedData);

				// Send a wind sensor message out

				MessagePtr windData = std::make_unique<WindDataMsg>(node->m_MeanWindDir, node->m_MeanWindSpeed, node->m_MeanWindTemp);
				node->m_MsgBus.sendMessage(std::move(windData));
			}
		}
	}
}

bool CV7Node::parseString(std::string& buffer_to_parse, float& windDir, float& windSpeed, float& windTemp) const
{
	const int IIMWV = 0;
	const int WIXDR = 1;
	bool updated[] = { false, false };
	/*"$IIMWV,%03.1f,R,%03.1f,M,*AB
	   $WIXDR,C,%03.1f,C,,*AB",*/
	std::regex iimwv_regex ("\\$IIMWV,([^,]{0,6}),.,([^,]{0,6}),.,[^\\$]{0,4}\\n?");
	std::regex wixdir_regex ("\\$WIXDR,.,([^,]{0,6}),.,.?,[^\\$]{0,4}");

  // erase everything before first $ -
  std::size_t found_begin = buffer_to_parse.find_first_of("$");
  buffer_to_parse.erase(0,found_begin);
	//----------------------------------

  if (buffer_to_parse.size()>0)
	{

		std::smatch sm;
		bool erase = false;

		/* while there still interesting data in the buffer*/
		while(std::regex_search (buffer_to_parse,iimwv_regex) || std::regex_search (buffer_to_parse,wixdir_regex))
		{
      erase = true;
			/*get the specific match string with regex groups (with smatch)*/
			if(std::regex_search( buffer_to_parse, sm,iimwv_regex ))
			{
				windDir = stof(sm[1].str()); //sm[1] is the first substring in parenthesis in the regular expression
				windSpeed = stof(sm[2].str());
				updated[IIMWV] = true;
			}
			else
			{
				std::regex_search ( buffer_to_parse, sm,wixdir_regex );
        windTemp = stof(sm[1].str());
				updated[WIXDR] = true;
			}
			buffer_to_parse.erase(sm.position(0),sm[0].str().size());//erase the match from buffer

		}
		if (erase)// if find match in buffer erase everything before the match
		{
		  buffer_to_parse.erase(0,sm.position(0));
		}
	}
	else
	{
		return false;
	}
	return (updated[IIMWV]); //|| updated[WIXDR]); we dont care much for temperature for the moment
}
