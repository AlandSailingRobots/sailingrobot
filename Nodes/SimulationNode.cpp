/****************************************************************************************
 *
 * File:
 * 		VesselStateNode.cpp
 *
 * Purpose:
 *		Maintains the "current" state of the vessel. Collects data from sensor messages
 *		and then resends a collected copy of that data back out for further processing.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "SimulationNode.h"

// For std::this_thread
#include <chrono>
#include <thread>

#include <sys/types.h>
#include <netdb.h>
#include <fcntl.h>
#include <strings.h> //bzero strerror
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include "SystemServices/Logger.h"
#include "utility/SysClock.h"


#define BASE_SLEEP_MS 400
#define COUNT_COMPASSDATA_MSG 1
#define COUNT_GPSDATA_MSG 1
#define COUNT_WINDDATA_MSG 1
#define COUNT_ARDUINO_MSG 1


SimulationNode::SimulationNode(MessageBus& msgBus)
	: ActiveNode(NodeID::VesselState, msgBus),
		m_CompassHeading(0), m_CompassPitch(0), m_CompassRoll(0),
		m_GPSHasFix(false), m_GPSOnline(false), m_GPSLat(0), m_GPSLon(0), m_GPSUnixTime(0), m_GPSSpeed(0),
		m_GPSHeading(0), m_WindDir(0), m_WindSpeed(0), m_WindTemp(0), m_ArduinoPressure(0),
		m_ArduinoRudder(0),m_ArduinoSheet(0),m_ArduinoBattery(0),m_rudder(0),m_sail(0),m_count_sleep(0)
{
  m_MsgBus.registerNode(this, MessageType::ActuatorPosition);
}

void SimulationNode::start()
{
	runThread(SimulationThreadFunc);
}

bool SimulationNode::init(){


		if (init_socket(6400)<0)
		{
			return false;
		}


		Logger::info("Size of float on this platform %d\n",(int)sizeof(float)*8 );
		Logger::info("Address: %s Port: %d\n", inet_ntoa(m_handler_socket_server.info_me.sin_addr),6400);

		//================
		unsigned int clntLen;            /* Length of client address data structure */
		/* Set the size of the in-out parameter */
		clntLen = sizeof(m_handler_socket_client.info_me);
		/* Wait for a client to connect */

		Logger::info("Waiting for simulation client...\n");

		if ((m_handler_socket_client.sockfd = accept(m_handler_socket_server.sockfd, (struct sockaddr *) &(m_handler_socket_client.info_me),
					 &clntLen)) < 0)
	  {
			  Logger::error("Error in accept() failed: %s",__PRETTY_FUNCTION__,strerror(errno));
				return false;
	  }

		fcntl(m_handler_socket_client.sockfd, F_SETFL, O_NONBLOCK);
    fcntl(m_handler_socket_server.sockfd, F_SETFL, O_NONBLOCK);
    Logger::info("Handling Simulation client from : %s\n", inet_ntoa(m_handler_socket_client.info_me.sin_addr));

		return true;
}

int SimulationNode::init_socket(int port)
{
    // init socket
    m_handler_socket_server.sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_handler_socket_server.sockfd == -1)
    {
				Logger::error("Error in initialise socket: %s",__PRETTY_FUNCTION__,strerror(errno));
        return -1;
    }

    // configure
    bzero((char *)&m_handler_socket_server.info_me, sizeof(m_handler_socket_server.info_me));
    m_handler_socket_server.info_me.sin_family = AF_INET;
    m_handler_socket_server.info_me.sin_port = htons(port);
    m_handler_socket_server.info_me.sin_addr.s_addr = htonl(INADDR_ANY);

    if( bind(m_handler_socket_server.sockfd, (struct sockaddr*) &m_handler_socket_server.info_me,
             sizeof(m_handler_socket_server.info_me)) == -1)
    {
        Logger::error("Error in initialise socket bind: %s",__PRETTY_FUNCTION__,strerror(errno));
        return -1;
    }

    if (listen(m_handler_socket_server.sockfd,1) < 0)
    {
        Logger::error("Error in initialise socket listen: %s",__PRETTY_FUNCTION__,strerror(errno));
        return -1;
    }
    return 0;
}


void SimulationNode::processMessage(const Message* msg)
{
	MessageType type = msg->messageType();

	switch(type)
	{
	case MessageType::ActuatorPosition:
		processActuatorPositionMessage((ActuatorPositionMsg*)msg);
		break;
	default:
		return;
	}
}

void SimulationNode::processActuatorPositionMessage(ActuatorPositionMsg* msg)
{
	m_rudder = msg->rudderPosition();
	m_sail = msg->sailPosition();
}

void SimulationNode::createCompassMessage()
{
	m_CompassHeading = m_data_receive.headingVector[0]/10.f;
	m_CompassPitch = 0;
	m_CompassRoll = 0;

	if (m_count_sleep % COUNT_COMPASSDATA_MSG==0){
	  CompassDataMsg* msg = new CompassDataMsg( int(m_CompassHeading+0.5) , m_CompassPitch, m_CompassRoll);
	  m_MsgBus.sendMessage(msg);
  }
}

void SimulationNode::createGPSMessage()
{
	double knots = 1.94384;
	m_GPSHasFix = true;
	m_GPSOnline = true;
	m_GPSLat = m_data_receive.latitude;
	m_GPSLon = m_data_receive.longitude;
	m_GPSUnixTime = SysClock::unixTime();
	m_GPSSpeed = m_data_receive.speed_knot/knots;
	m_GPSHeading = m_data_receive.course_real;
	m_GPSSatellite = 0;
	GPSMode mode = GPSMode::LatLonOk;

	if (m_count_sleep % COUNT_GPSDATA_MSG==0)
	{
		GPSDataMsg* msg = new GPSDataMsg(m_GPSHasFix, m_GPSOnline, m_GPSLat, m_GPSLon, m_GPSUnixTime, m_GPSSpeed, m_GPSHeading, m_GPSSatellite, mode);
		m_MsgBus.sendMessage(msg);
	}
}

void SimulationNode::createWindMessage()
{
	m_WindDir = m_data_receive.windDirection;
	m_WindSpeed = m_data_receive.windSpeed;
	m_WindTemp = m_data_receive.windTemperature;

	if (m_count_sleep % COUNT_WINDDATA_MSG==0)
	{
		WindDataMsg* windData = new WindDataMsg(m_WindDir, m_WindSpeed, m_WindTemp);
		m_MsgBus.sendMessage(windData);
	}
}

void SimulationNode::createArduinoMessage()
{
	m_ArduinoPressure = m_data_receive.pressure;
	m_ArduinoRudder = m_data_receive.rudder;
	m_ArduinoSheet = m_data_receive.sheet;
	m_ArduinoBattery = m_data_receive.battery;

	if (m_count_sleep % COUNT_ARDUINO_MSG==0)
	{
		ArduinoDataMsg* msg = new ArduinoDataMsg(m_ArduinoPressure, m_ArduinoRudder, m_ArduinoSheet, m_ArduinoBattery );
		m_MsgBus.sendMessage(msg);
	}
}

void SimulationNode::processSocketData(){

  createCompassMessage();
  createGPSMessage();
  createWindMessage();
  createArduinoMessage();
}

void SimulationNode::setupDataSend(){
  m_data_send.rudder_command = m_rudder;
  m_data_send.sheet_command = m_sail;
}

void SimulationNode::SimulationThreadFunc(void* nodePtr)
{
	SimulationNode* node = (SimulationNode*)nodePtr;

	int bytes_received = 0;
	struct DATA_SOCKET_RECEIVE dump_data_sock_receive;
	while(true){

    //receive socket from simulation
    bytes_received += read(node->m_handler_socket_client.sockfd,&(node->m_data_receive)+bytes_received,sizeof(struct DATA_SOCKET_RECEIVE)-bytes_received);
    if (bytes_received==sizeof(struct DATA_SOCKET_RECEIVE)){
       bytes_received=0;
       while(read(node->m_handler_socket_client.sockfd,&dump_data_sock_receive,sizeof(struct DATA_SOCKET_RECEIVE))>0){};
    }
    if (bytes_received==-1)
      bytes_received=0;

    //clock for sending messages
    if (node->m_count_sleep==10)
		{
			node->m_count_sleep=0;
		}
		node->m_count_sleep++;

    node->processSocketData();
		node->setupDataSend();

    write(node->m_handler_socket_client.sockfd,&(node->m_data_send), sizeof(struct DATA_SOCKET_SEND));

		std::this_thread::sleep_for(std::chrono::milliseconds(BASE_SLEEP_MS));
  }
}
