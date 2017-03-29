/****************************************************************************************
 *
 * File:
 * 		SimulationNode.cpp
 *
 * Purpose:
 *		Discuss with simulation via TCP, create message for the program from the
 *    data from simulation and send the command data to the simulation.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "SimulationNode.h"

// For std::this_thread
#include <chrono>
#include <thread>
#include <memory>

#include <sys/types.h>
#include <netdb.h>
#include <fcntl.h>
#include <strings.h> //bzero strerror
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include "SystemServices/Logger.h"
#include "SystemServices/SysClock.h"
#include "Network/TCPServer.h"


#define BASE_SLEEP_MS 400
#define COUNT_COMPASSDATA_MSG 1
#define COUNT_GPSDATA_MSG 1
#define COUNT_WINDDATA_MSG 1
#define COUNT_ARDUINO_MSG 1

#define SERVER_PORT	6900


enum SimulatorPacket {
	BoatData = 0,
	AISData = 1,
	CameraData = 2
};


SimulationNode::SimulationNode(MessageBus& msgBus)
	: ActiveNode(NodeID::Simulator, msgBus),
		m_CompassHeading(0), m_GPSLat(0), m_GPSLon(0), m_GPSSpeed(0),
		m_GPSHeading(0), m_WindDir(0), m_WindSpeed(0),
		m_ArduinoRudder(0), m_ArduinoSheet(0)
{
  m_MsgBus.registerNode(*this, MessageType::ActuatorPosition);
}

void SimulationNode::start()
{
	runThread(SimulationThreadFunc);
}

bool SimulationNode::init()
{
	bool success = false;

	int rc = server.start( SERVER_PORT );

	if( rc > 0 )
	{
		Logger::info("Waiting for simulation client...\n");

		// Block until connection, don't timeout
		server.acceptConnection( 0 );

		success = true;
	}
	else
	{
		Logger::error( "Failed to start the simulator server" );
		success = false;
	}

	return success;
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
	actuatorData.rudderCommand = msg->rudderPosition();
	actuatorData.sailCommand = msg->sailPosition();
}

void SimulationNode::createCompassMessage()
{
	MessagePtr msg = std::make_unique<CompassDataMsg>(CompassDataMsg( m_CompassHeading, 0, 0));
	m_MsgBus.sendMessage(std::move(msg));
}

void SimulationNode::createGPSMessage()
{
	MessagePtr msg = std::make_unique<GPSDataMsg>(GPSDataMsg(true, true, m_GPSLat, m_GPSLon, SysClock::unixTime(), m_GPSSpeed, m_GPSHeading, 0, GPSMode::LatLonOk));
	m_MsgBus.sendMessage(std::move(msg));
}

void SimulationNode::createWindMessage()
{
	MessagePtr windData = std::make_unique<WindDataMsg>( WindDataMsg(m_WindDir, m_WindSpeed, 21) );
	m_MsgBus.sendMessage( std::move(windData) );
}

void SimulationNode::createArduinoMessage()
{
	MessagePtr msg = std::make_unique<ArduinoDataMsg>(ArduinoDataMsg(0, m_ArduinoRudder, m_ArduinoSheet, 0, 0 ));
	m_MsgBus.sendMessage(std::move(msg));
}

///--------------------------------------------------------------------------------------
void SimulationNode::processBoatData( TCPPacket_t& packet )
{	
	if( packet.length - 1 == sizeof(BoatDataPacket_t) )
	{
		// The first byte is the packet type, lets skip that
		uint8_t* ptr = packet.data + 1;
		BoatDataPacket_t* boatData = (BoatDataPacket_t*)ptr;

		m_CompassHeading = boatData->heading;
		m_GPSLat = boatData->latitude;
		m_GPSLon = boatData->longitude;
		m_GPSSpeed = boatData->speed;
		m_GPSHeading = boatData->course;
		m_WindDir = boatData->windDir;
		m_WindSpeed = boatData->windSpeed;
	 	m_ArduinoRudder = boatData->rudder;
	 	m_ArduinoSheet = boatData->sail;

		 Logger::info("Heading: %d Lat: %f Lon: %f", m_CompassHeading, m_GPSLat, m_GPSLon);

		// Send messages
		createCompassMessage();
		createGPSMessage();
		createWindMessage();
		createArduinoMessage();
	}
}

///--------------------------------------------------------------------------------------
void SimulationNode::sendActuatorData( int socketFD )
{
	server.sendData( socketFD, &actuatorData, sizeof(ActuatorDataPacket_t) );
}

///--------------------------------------------------------------------------------------
void SimulationNode::SimulationThreadFunc(void* nodePtr)
{
	SimulationNode* node = (SimulationNode*)nodePtr;

	TCPPacket_t packet;
	int simulatorFD = 0;

	while(true)
	{
		// We only care about the latest packet, so clear out the old ones
		node->server.clearSocketBuffer( packet.socketFD );

		// Don't timeout on a packet read
		node->server.readPacket( packet, 0 );

		// We can safely assume that the first packet we receive will actually be from
		// the simulator as we only should ever accept one connection, the first one/
		if( simulatorFD == 0 )
		{
			simulatorFD = packet.socketFD;
		}

		// First byte is the message type
		switch( packet.data[0] )
		{
			case SimulatorPacket::BoatData:
				node->processBoatData( packet );
			break;

			case SimulatorPacket::AISData:
				// TODO
			break;

			case SimulatorPacket::CameraData:
				// TODO
			break;

			// unknown or deformed packet
			default:
				continue;
		}
		
		node->sendActuatorData( simulatorFD );

		// Reset our packet, better safe than sorry
		packet.socketFD = 0;
		packet.length = 0;

		std::this_thread::sleep_for(std::chrono::milliseconds(BASE_SLEEP_MS));
	}
}
