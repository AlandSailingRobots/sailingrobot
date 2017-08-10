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

#include "VesselStateNode.h"

// For std::this_thread
#include <chrono>
#include <thread>

#include "Messages/VesselStateMsg.h"
#include "Math/CourseMath.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"





VesselStateNode::VesselStateNode(MessageBus& msgBus, double loopTime)
	: ActiveNode(NodeID::VesselState, msgBus),
		m_CompassHeading(0), m_CompassPitch(0), m_CompassRoll(0),
		m_GPSHasFix(false), m_GPSOnline(false), m_GPSLat(0), m_GPSLon(0), m_GPSUnixTime(0), m_GPSSpeed(0),
		m_GPSCourse(0), m_WindDir(0), m_WindSpeed(0), m_WindTemp(0), m_ArduinoPressure(0),
		m_ArduinoRudder(0),m_ArduinoSheet(0),m_ArduinoBattery(0), m_LoopTime(loopTime)
{

	msgBus.registerNode(*this, MessageType::CompassData);
	msgBus.registerNode(*this, MessageType::GPSData);
	msgBus.registerNode(*this, MessageType::WindData);
	msgBus.registerNode(*this, MessageType::ArduinoData);
	msgBus.registerNode(*this, MessageType::WaypointData);
}

//---------------------------------------------------------------------------------------
VesselStateNode::~VesselStateNode()
{
	server.shutdown();
}

//---------------------------------------------------------------------------------------
bool VesselStateNode::init()
{
	return server.start( 9600 );
}

void VesselStateNode::start()
{
	runThread(VesselStateThreadFunc);
}

void VesselStateNode::processMessage(const Message* msg)
{
	MessageType type = msg->messageType();

	switch(type)
	{
	case MessageType::CompassData:
		processCompassMessage((CompassDataMsg*)msg);
		break;
	case MessageType::GPSData:
		processGPSMessage((GPSDataMsg*)msg);
		break;
	case MessageType::WindData:
		processWindMessage((WindDataMsg*)msg);
		break;
	case MessageType::ArduinoData:
		processArduinoMessage((ArduinoDataMsg*)msg);
		break;
	case MessageType::WaypointData:
		processWaypointMessage((WaypointDataMsg*)msg);
	default:
		return;
	}
}

void VesselStateNode::processCompassMessage(CompassDataMsg* msg)
{
	m_CompassHeading = msg->heading();
	m_CompassPitch = msg->pitch();
	m_CompassRoll = msg->roll();
}

void VesselStateNode::processGPSMessage(GPSDataMsg* msg)
{
	m_GPSHasFix = msg->hasFix();
	m_GPSOnline = msg->gpsOnline();
	m_GPSLat = msg->latitude();
	m_GPSLon = msg->longitude();
	m_GPSUnixTime = msg->unixTime();
	m_GPSSpeed = msg->speed();
	m_GPSCourse = msg->course();
	m_GPSSatellite = msg->satelliteCount();
	waypointBearing = CourseMath::calculateBTW( m_GPSLon, m_GPSLat, waypointLon, waypointLat );
	waypointDistance = CourseMath::calculateDTW( m_GPSLon, m_GPSLat, waypointLon, waypointLat );
}

void VesselStateNode::processWindMessage(WindDataMsg* msg)
{
	m_WindDir = msg->windDirection();
	m_WindSpeed = msg->windSpeed();
	m_WindTemp = msg->windTemp();
}

void VesselStateNode::processArduinoMessage(ArduinoDataMsg* msg)
{
	m_ArduinoPressure = msg->pressure();
	m_ArduinoRudder = msg->rudder();
	m_ArduinoSheet = msg->sheet();
	m_ArduinoBattery = msg->battery();
	m_ArduinoRC = msg->Radio_Controller();
}

void VesselStateNode::processWaypointMessage( WaypointDataMsg* msg )
{
	waypointID = msg->nextId();
	waypointLat = msg->nextLatitude();
	waypointLon = msg->nextLongitude();

	radius = msg->nextRadius();
	waypointBearing = CourseMath::calculateBTW( m_GPSLon, m_GPSLat, waypointLon, waypointLat );
	waypointDistance = CourseMath::calculateDTW( m_GPSLon, m_GPSLat, waypointLon, waypointLat );
}

void VesselStateNode::VesselStateThreadFunc(ActiveNode* nodePtr)
{
	VesselStateNode* node = dynamic_cast<VesselStateNode*> (nodePtr);

	// An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
	// at the start before we send out the vessel state message.
	std::this_thread::sleep_for(std::chrono::milliseconds(node->VESSEL_STATE_INITIAL_SLEEP));

	char buffer[1024];

	Timer timer;
	timer.start();
	while(true)
	{
		// Listen to new connections
		node->server.acceptConnections();

		// Controls how often we pump out messages
		timer.sleepUntil(node->m_LoopTime);

		MessagePtr vesselState = std::make_unique<VesselStateMsg>(	node->m_CompassHeading, node->m_CompassPitch,
																	node->m_CompassRoll, node->m_GPSHasFix, node->m_GPSOnline, node->m_GPSLat,
																	node->m_GPSLon, node->m_GPSUnixTime, node->m_GPSSpeed, node->m_GPSSatellite,
																	node->m_GPSCourse, node->m_WindDir, node->m_WindSpeed,
																	node->m_WindTemp, node->m_ArduinoPressure, node->m_ArduinoRudder,
																	node->m_ArduinoSheet, node->m_ArduinoBattery, node->m_ArduinoRC);
		node->m_MsgBus.sendMessage(std::move(vesselState));

		// Compass heading, Compass Pitch, Compass Roll, Wind Dir, Wind Speed, GPS Course, GPS Lat, GPS Lon
		int size = snprintf( buffer, 1024, "%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%f,%f,%d,%f,%d\n", (int)node->m_CompassHeading, (int)node->m_CompassPitch, (int)node->m_CompassRoll, (int)node->m_WindDir, (int)node->m_WindSpeed, (int)node->m_GPSCourse, (float)node->m_GPSSpeed, node->m_GPSLat, node->m_GPSLon, node->waypointID, node->waypointLat, node->waypointLon, (int)node->radius, node->waypointDistance, (int)node->waypointBearing );
		if( size > 0 )
		{
			node->server.broadcast( (uint8_t*)buffer, size );
		}
		timer.reset();
	}
}
