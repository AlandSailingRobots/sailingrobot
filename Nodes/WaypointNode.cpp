#include "WaypointNode.h"
#include "Messages/WaypointDataMsg.h"
#include "logger/Logger.h"
#include "dbhandler/DBHandler.h"

WaypointNode::WaypointNode(MessageBus& msgBus)
: ActiveNode(NodeID::Waypoint, msgBus)
{

}


void WaypointNode::processMessage(const Message *msg)
{

}

void WaypointNode::getWaypointValues()
{

}