/****************************************************************************************
 *
 * File:
 * 		MarineSensorDataRequestMsg.h
 *
 * Purpose:
 *		A MarineSensorDataRequestMsg sends a request for marine sensor data top be read.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"

class MarineSensorDataRequestMsg : public Message
{
public:
    MarineSensorDataRequestMsg(NodeID destinationID, NodeID sourceID)
            :Message(MessageType::MarineSensorDataRequest, sourceID, destinationID)
    { }

    MarineSensorDataRequestMsg()
            :Message(MessageType::MarineSensorDataRequest, NodeID::None, NodeID::None)
    { }

    MarineSensorDataRequestMsg(MessageDeserialiser& deserialiser)
            :Message(deserialiser)
    {
    }

    virtual ~MarineSensorDataRequestMsg() { }

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const
    {
        Message::Serialise(serialiser);
    }


};
