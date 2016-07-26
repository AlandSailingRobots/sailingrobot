/****************************************************************************************
 *
 * File:
 * 		WaypointNode.cpp
 *
 * Purpose:
 *		The WaypointNode sends information about the waypoints to the sailing logic
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "WaypointNode.h"
#include "Messages/WaypointDataMsg.h"
#include "SystemServices/Logger.h"
#include <string>
#include <vector>

WaypointNode::WaypointNode(MessageBus& msgBus, DBHandler& db)
: Node(NodeID::Waypoint, msgBus), m_db(db), 
    m_id(0),
    m_longitude(0),
    m_latitude(0),
    m_declination (0),
    m_radius(0),
    m_gps_longitude(0),
    m_gps_latitude(0)
{

}

bool WaypointNode::init()
{ 
    sendMessage();
    return true;
}


void WaypointNode::processMessage(const Message* msg)
{
	MessageType type = msg->messageType();

	switch(type)
	{
        case MessageType::GPSData:
            processGPSMessage((GPSDataMsg*)msg);
            break;
        default:
            return;
	}
    
    //TODO Oliver - ADD waypoint time to message
    if(waypointReached()/* || httpsync sent info that waypoints have been updated*/)
    {
        sendMessage();
    }
}

void WaypointNode::processGPSMessage(GPSDataMsg* msg)
{
    m_gps_longitude = msg->longitude();
    m_gps_latitude = msg->latitude();
}

bool WaypointNode::waypointReached()
{
    if(m_courseMath.calculateDTW(m_gps_longitude, m_gps_latitude, m_longitude, m_latitude) < m_radius)
    {
        if(not m_db.changeOneValue("waypoints", std::to_string(m_id),"1","harvested"))
        {
            Logger::error("Failed to harvest waypoint");
        }
        Logger::info("Reached waypoint");

        return true;
    }
    else
    {
        return false;
    }
}

void WaypointNode::sendMessage()
{
    Logger::info("Preparing to send WaypointNode");

    if(m_db.getWaypointValues(m_id, m_longitude, m_latitude, m_declination, m_radius))
    {
        WaypointDataMsg* msg = new WaypointDataMsg(m_id, m_longitude, m_latitude, m_declination, m_radius);
        m_MsgBus.sendMessage(msg);
    }
    else
    {
        Logger::warning("%s No waypoint found, boat is using old waypoint data. No message sent.", __func__);
    }
}
