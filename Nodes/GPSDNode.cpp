/****************************************************************************************
 *
 * File:
 * 		GPSDNode.cpp
 *
 * Purpose:
 *		A GPSD node that uses the GPSD library to provide GPS data to the message bus.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "GPSDNode.h"


GPSDNode::GPSDNode(MessageBus& msgBus)
	: ActiveNode(NodeID::GPS, msgBus), m_Initialised(false), m_GpsConnection(0)
{

}

bool GPSDNode::init()
{
	m_Initialised = false;
	m_GpsConnection = new gpsmm("localhost", DEFAULT_GPSD_PORT);

	if (m_GpsConnection->stream(WATCH_ENABLE | WATCH_JSON) != NULL) {
		m_Initialised = true;
	}

	return m_Initialised;
}

void GPSDNode::processMessage(Message* msgPtr)
{

}

void GPSDNode::start()
{
	if(m_Initialised)
	{
		runThread(WindSensorThread);
	}
	else
	{
		Logger::error("%s Cannot start GPSD sensor thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
}

void GPSDNode::GPSThreadFunc(void* nodePtr)
{

}
