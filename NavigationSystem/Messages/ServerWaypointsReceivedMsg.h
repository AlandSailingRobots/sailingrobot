
/****************************************************************************************
 * Purpose:
 *		Notify receivers that new waypoints have been downloaded from the server
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "../MessageBus/Message.h"

class ServerWaypointsReceivedMsg : public Message {
   public:
    ServerWaypointsReceivedMsg(NodeID destinationID, NodeID sourceID)
        : Message(MessageType::ServerWaypointsReceived, sourceID, destinationID) {}

    ServerWaypointsReceivedMsg()
        : Message(MessageType::ServerWaypointsReceived, NodeID::None, NodeID::None) {}

    ServerWaypointsReceivedMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {}

    virtual ~ServerWaypointsReceivedMsg() {}
};
