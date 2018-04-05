/****************************************************************************************
 *
 * File:
 * 		DataCollectionStopMsg.h
 *
 * Purpose:
 *		A DataCollectionStopMsg is used to stop automatic reading from marine sensors
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"

class DataCollectionStopMsg : public Message {
public:
    DataCollectionStopMsg(NodeID sourceID, NodeID destinationID)
            :Message(MessageType::DataCollectionStop, sourceID, destinationID)
    {

    }

    DataCollectionStopMsg(NodeID destinationID)
            :Message(MessageType::DataCollectionStop, NodeID::None, destinationID)
    {

    }

    DataCollectionStopMsg(MessageDeserialiser deserialiser)
            :Message(deserialiser)
    { }

    virtual ~DataCollectionStopMsg() { }
};
