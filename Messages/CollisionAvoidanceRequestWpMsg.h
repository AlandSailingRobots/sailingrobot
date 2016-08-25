
/****************************************************************************************
 * Purpose:
 *		Notify waypoint manager to send gps the waypoints to collision avoidance
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"

class CollisionAvoidanceRequestWpMsg : public Message {
public:
	CollisionAvoidanceRequestWpMsg(NodeID destinationID, NodeID sourceID)
		:Message(MessageType::CollisionAvoidanceRequestWp, sourceID, destinationID)
	{ }

	CollisionAvoidanceRequestWpMsg()
		:Message(MessageType::CollisionAvoidanceRequestWp, NodeID::None, NodeID::None)
	{ }

	CollisionAvoidanceRequestWpMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{ }

	virtual ~CollisionAvoidanceRequestWpMsg() { }

};
