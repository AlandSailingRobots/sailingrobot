/****************************************************************************************
*
* File:
* 		StateEstimationNode.cpp
*
* Purpose:
* Maintains the "current" state listening to GPS and compass messages, sending out a
* StateMessage
*
* Developer Notes:
*
*
***************************************************************************************/

#include "StateEstimationNode.h"

// For std::this_thread
#include <chrono>
#include <thread>

#include "Messages/StateMessage.h"
#include "Math/CourseMath.h"
//#include "SystemServices/Logger.h"

#define STATE_SLEEP_MS 400
#define STATE_INITIAL_SLEEP 2000


StateEstimationNode::StateEstimationNode(MessageBus& msgBus): ActiveNode(NodeID::StateEstimation, msgBus),
vesselHeading(0), vesselLat(0), vesselLan(0),vesselSpeed(0)
{
  msgBus.registerNode(*this, MessageType::CompassData);
  msgBus.registerNode(*this, MessageType::GPSData);
}

StateEstimationNode::~StateEstimationNode()
{
  server.shutdown();
}

bool StateEstimationNode::init()
{
  return server.start( 9600 );
}

void StateEstimationNode::start()
{
  runThread(StateEstimationNodeThreadFunc);
}

void StateEstimationNode::processMessage(const Message* msg)
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
    default:
    return;
  }
}

void StateEstimationNode::processCompassMessage(CompassDataMsg* msg)
{
	vesselHeading = msg->heading();
}

void StateEstimationNode::processGPSMessage(GPSDataMsg* msg)
{
	vesselLat = msg->latitude();
	vesselLan = msg->longitude();
	vesselSpeed = msg->speed();
	vesselHeading = msg->heading();
}

void StateEstimationNode::StateEstimationNodeThreadFunc(void* nodePtr)
{
	StateEstimationNode* node = (StateEstimationNode*)nodePtr;

	// An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
	// at the start before we send out the vessel state message.
	std::this_thread::sleep_for(std::chrono::milliseconds(STATE_INITIAL_SLEEP));

	char buffer[1024];

	while(true)
	{
		// Listen to new connections
		node->server.acceptConnections();

		// Controls how often we pump out messages
		std::this_thread::sleep_for(std::chrono::milliseconds(STATE_SLEEP_MS));

		MessagePtr stateMessage = std::make_unique<StateMessage>(	node->vesselHeading, node->vesselLat,
																	node->vesselLan, node->vesselSpeed);
		node->m_MsgBus.sendMessage(std::move(stateMessage));

		// Compass heading, Compass Pitch, Compass Roll, Wind Dir, Wind Speed, GPS Heading, GPS Lat, GPS Lon
		int size = snprintf( buffer, 1024, "%d,%f,%f,%f,%d\n",
    (int)node->vesselHeading,
    (float)node->vesselSpeed,
    node->vesselLat,
    node->vesselLan,
    (int)node->radius);
		if( size > 0 )
		{
			node->server.broadcast( (uint8_t*)buffer, size );
		}
	}
}
