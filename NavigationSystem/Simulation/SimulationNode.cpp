/****************************************************************************************
 *
 * File:
 *      SimulationNode.cpp
 *
 * Purpose:
 *      Discuss with simulation via TCP, create message for the program from the
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
#include <stdlib.h>
#include <sys/types.h>
#include <netdb.h>
#include <fcntl.h>
#include <strings.h> //bzero strerror
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <stdlib.h>
#include "SystemServices/Timer.h"
#include "SystemServices/Logger.h"
#include "SystemServices/SysClock.h"
#include "Network/TCPServer.h"
#include "Math/CourseMath.h"
#include "Math/Utility.h"


#define BASE_SLEEP_MS 200
#define COUNT_COMPASSDATA_MSG 1
#define COUNT_GPSDATA_MSG 1
#define COUNT_WINDDATA_MSG 1

#define SERVER_PORT 6900



SimulationNode::SimulationNode(MessageBus& msgBus, DBHandler& dbhandler)
	: ActiveNode(NodeID::Simulator, msgBus),
        m_RudderCommand(0), m_SailCommand(0), m_TailCommand(0),
		m_CompassHeading(0), m_GPSLat(0), m_GPSLon(0), m_GPSSpeed(0),
		m_GPSCourse(0), m_WindDir(0), m_WindSpeed(0), m_nextDeclination(0),
		collidableMgr(NULL),m_LoopTime(0.5), m_db(dbhandler)
{
    msgBus.registerNode(*this, MessageType::WingSailCommand);
    msgBus.registerNode(*this, MessageType::RudderCommand);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

SimulationNode::SimulationNode(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr)
	: ActiveNode(NodeID::Simulator, msgBus),
        m_RudderCommand(0), m_SailCommand(0), m_TailCommand(0),
		m_CompassHeading(0), m_GPSLat(0), m_GPSLon(0), m_GPSSpeed(0),
		m_GPSCourse(0), m_WindDir(0), m_WindSpeed(0), m_nextDeclination(0),
		collidableMgr(collidableMgr), m_LoopTime(0.5), m_db(dbhandler)
{
    msgBus.registerNode(*this, MessageType::WingSailCommand);
    msgBus.registerNode(*this, MessageType::RudderCommand);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

void SimulationNode::start()
{
    runThread(SimulationThreadFunc);
}

bool SimulationNode::init()
{
    bool success = false;
    updateConfigsFromDB();

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

void SimulationNode::updateConfigsFromDB(){
	m_LoopTime = m_db.retrieveCellAsDouble("config_simulator","1","loop_time");
}

void SimulationNode::processMessage(const Message* msg)
{
    MessageType type = msg->messageType();

    switch(type)
    {
    case MessageType::ActuatorPosition:
        processActuatorPositionMessage((ActuatorPositionMsg*)msg);
        break;
    case MessageType::WingSailCommand:
        // Logger::info("WingSailCommand message received");
        processWingSailCommandMessage((WingSailCommandMsg*)msg);
        break;
    case MessageType::RudderCommand:
        // Logger::info("RudderCommand message received");
        processRudderCommandMessage((RudderCommandMsg*)msg);
        break;
    case MessageType::WaypointData:
        Logger::info("WaypointData message received");
        processWaypointMessage((WaypointDataMsg*) msg);
        break;
    case MessageType::ServerConfigsReceived:
		updateConfigsFromDB();
	    break;
    default:
        return;
    }
}

void SimulationNode::processActuatorPositionMessage(ActuatorPositionMsg* msg)
{
    //m_RudderCommand = msg->rudderPosition();
    //m_SailCommand = msg->sailPosition();
}
void SimulationNode::processWingSailCommandMessage(WingSailCommandMsg* msg)
{
    m_TailCommand = msg->tailAngle();
}

void SimulationNode::processRudderCommandMessage(RudderCommandMsg* msg)
{
    m_RudderCommand = msg->rudderAngle();
}

void SimulationNode::processWaypointMessage(WaypointDataMsg* msg)
{
	waypoint.nextId = msg->nextId();
	waypoint.nextLongitude = msg->nextLongitude();
	waypoint.nextLatitude = msg->nextLatitude();
	waypoint.nextDeclination = msg->nextDeclination();
	waypoint.nextRadius = msg->nextRadius();
	waypoint.nextStayTime = msg->stayTime();
	waypoint.prevId = msg->prevId();
	waypoint.prevLongitude = msg->prevLongitude();
	waypoint.prevLatitude = msg->prevLatitude();
	waypoint.prevDeclination = msg->prevDeclination();
	waypoint.prevRadius = msg->prevRadius();

    m_nextDeclination = msg->nextDeclination();

	Logger::info("In processmessage, lat: " + std::to_string(waypoint.nextLatitude) + ", Lon: " + std::to_string(waypoint.nextLongitude));
}

void SimulationNode::createCompassMessage()
{
    MessagePtr msg = std::make_unique<CompassDataMsg>(CompassDataMsg( m_CompassHeading, 0, 0));
    m_MsgBus.sendMessage(std::move(msg));
}

void SimulationNode::createGPSMessage()
{
    MessagePtr msg = std::make_unique<GPSDataMsg>(GPSDataMsg(true, true, m_GPSLat, m_GPSLon, SysClock::unixTime(), m_GPSSpeed, m_GPSCourse, 0, GPSMode::LatLonOk));
    m_MsgBus.sendMessage(std::move(msg));
}

void SimulationNode::createWindMessage()
{
    MessagePtr windData = std::make_unique<WindDataMsg>( WindDataMsg(m_WindDir, m_WindSpeed, 21) );
    m_MsgBus.sendMessage( std::move(windData) );
}

void SimulationNode::createASPireActuatorFeedbackMessage()
{
    MessagePtr ASPireActuatorFeedback = std::make_unique<ASPireActuatorFeedbackMsg>( ASPireActuatorFeedbackMsg(-2000, -2000, -2000, -2000, 0) );
    m_MsgBus.sendMessage( std::move(ASPireActuatorFeedback) );
}


///--------------------------------------------------------------------------------------
void SimulationNode::processSailBoatData( TCPPacket_t& packet )
{
    if( packet.length - 1 == sizeof(SailBoatDataPacket_t) )
    {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        SailBoatDataPacket_t* boatData = (SailBoatDataPacket_t*)ptr;

        m_CompassHeading = Utility::limitAngleRange(90 - boatData->heading); // [0, 360] north east down
        m_GPSLat = boatData->latitude;
        m_GPSLon = boatData->longitude;
        m_GPSSpeed = boatData->speed;
        m_GPSCourse = Utility::limitAngleRange(90 - boatData->course); // [0, 360] north east down
        m_WindDir = Utility::limitAngleRange( 180 - boatData->windDir); // [0, 360] clockwize, where the wind come from
        m_WindSpeed = boatData->windSpeed;

        // Send messages
        createCompassMessage();
        createGPSMessage();
        createWindMessage();
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::processWingBoatData( TCPPacket_t& packet )
{
    if( packet.length - 1 == sizeof(WingBoatDataPacket_t) )
    {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        WingBoatDataPacket_t* boatData = (WingBoatDataPacket_t*)ptr;

        m_CompassHeading = Utility::limitAngleRange(90 - boatData->heading - m_nextDeclination); // [0, 360] north east down
        m_GPSLat = boatData->latitude;
        m_GPSLon = boatData->longitude;
        m_GPSSpeed = boatData->speed;
        m_GPSCourse = Utility::limitAngleRange(90 - boatData->course); // [0, 360] north east down
        m_WindDir = Utility::limitAngleRange(180 - boatData->windDir); // [0, 360] clockwize, where the wind come from
        m_WindSpeed = boatData->windSpeed;
        // std::cout <<"heading " << m_CompassHeading << std::endl;
        // std::cout <<"lat " << m_GPSLat << std::endl;
        // std::cout <<"long " << m_GPSLon << std::endl;
        // std::cout <<"speed " << m_GPSSpeed << std::endl;
        // std::cout <<"course " << m_GPSCourse << std::endl;
        // std::cout <<"windSpeed " << m_WindSpeed << std::endl;
        // std::cout <<"WindDir " << m_WindDir << std::endl;

        // Send messages
        createCompassMessage();
        createGPSMessage();
        createWindMessage();
        createASPireActuatorFeedbackMessage();
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::processAISContact( TCPPacket_t& packet )
{
	if( this->collidableMgr != NULL )
	{
		// The first byte is the packet type, lets skip that
		uint8_t* ptr = packet.data + 1;
		AISContactPacket_t* aisData = (AISContactPacket_t*)ptr;

		this->collidableMgr->addAISContact(aisData->mmsi, aisData->latitude, aisData->longitude, aisData->speed, Utility::limitAngleRange(90 - aisData->course) /* [0, 360] north east down*/);
		this->collidableMgr->addAISContact(aisData->mmsi, aisData->length, aisData->beam);
	}
}

///--------------------------------------------------------------------------------------
void SimulationNode::processVisualContact( TCPPacket_t& packet )
{
    if( this->collidableMgr != NULL )
    {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        VisualContactPacket_t* data = (VisualContactPacket_t*)ptr;

        uint16_t bearing = CourseMath::calculateBTW(m_GPSLon, m_GPSLat, data->longitude, data->latitude);

        this->collidableMgr->addVisualContact(data->id, bearing);
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::sendActuatorDataWing( int socketFD)
{
    // m_RudderCommand = 0;
    // m_TailCommand   = 15.0;
    actuatorDataWing.rudderCommand = - Utility::degreeToRadian(m_RudderCommand);
    actuatorDataWing.tailCommand   = - Utility::degreeToRadian(m_TailCommand);
    // std::cout <<"sent rudder command " << actuatorDataWing.rudderCommand << std::endl;
    // std::cout <<"sent tail command " << actuatorDataWing.tailCommand << std::endl;
    // std::cout <<"given rudder command " << m_RudderCommand << std::endl;
    // std::cout <<"given tail command " << m_TailCommand << std::endl;

    server.sendData( socketFD, &actuatorDataWing, sizeof(ActuatorDataWingPacket_t) );
}

void SimulationNode::sendActuatorDataSail( int socketFD)
{   
    actuatorDataSail.rudderCommand = - Utility::degreeToRadian(m_RudderCommand);
    actuatorDataSail.sailCommand   = - Utility::degreeToRadian(m_SailCommand);
    server.sendData( socketFD, &actuatorDataSail, sizeof(ActuatorDataSailPacket_t) );       
}

///--------------------------------------------------------------------------------------
void SimulationNode::sendWaypoint( int socketFD )
{
	server.sendData( socketFD, &waypoint, sizeof(WaypointPacket_t) );
}

///--------------------------------------------------------------------------------------
void SimulationNode::SimulationThreadFunc(ActiveNode* nodePtr)
{
    SimulationNode* node = dynamic_cast<SimulationNode*> (nodePtr);

    //Timer timer;
    //timer.start();

    TCPPacket_t packet;
    int simulatorFD = 0;

    while(true)
    {
        // Don't timeout on a packet read
        node->server.readPacket( packet, 0 );

        // We only care about the latest packet, so clear out the old ones
        //node->server.clearSocketBuffer( packet.socketFD );

        // We can safely assume that the first packet we receive will actually be from
        // the simulator as we only should ever accept one connection, the first one/
        if( simulatorFD == 0 )
        {
            simulatorFD = packet.socketFD;
        }
        // First byte is the message type
        switch( packet.data[0] )
        {

            case SimulatorPacket::SailBoatData:
                node->processSailBoatData( packet );
                //std::cout<<"SailBoatData"<< std::endl;
                break;

            case SimulatorPacket::WingBoatData:
                node->processWingBoatData( packet );
                //std::cout<<"WingBoatData"<< std::endl;
                break;
            case SimulatorPacket::AISData:
                node->processAISContact( packet );
                break;

            case SimulatorPacket::CameraData:
                node->processVisualContact( packet );
                break;

            // unknown or deformed packet
            default:
                continue;
        }
        // Reset our packet, better safe than sorry
        packet.socketFD = 0;
        packet.length = 0;

        node->sendActuatorDataWing ( simulatorFD );
        //node->sendActuatorDataSail( simulatorFD);
        node->sendWaypoint( simulatorFD );

        //timer.sleepUntil(node->m_LoopTime);
        //timer.reset();
    }
}
