/****************************************************************************************
 *
 * File:
 * 		DataCollectionStartMsg.h
 *
 * Purpose:
 *		A DataCollectionStartMsg starts automatic readings from marine sensors on a certain interval
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/
#pragma once

#include "MessageBus/Message.h"


class DataCollectionStartMsg : public Message {
public:
    DataCollectionStartMsg(NodeID sourceID, NodeID destinationID, int sensorReadingInterval)
            :Message(MessageType::DataCollectionStart, sourceID, destinationID), m_sensorReadingInterval(sensorReadingInterval)
    {

    }

    DataCollectionStartMsg(int sensorReadingInterval)
            :Message(MessageType::DataCollectionStart, NodeID::None, NodeID::None), m_sensorReadingInterval(sensorReadingInterval)
    {

    }

    DataCollectionStartMsg(MessageDeserialiser deserialiser) : Message(deserialiser)
    {
        if(	!deserialiser.readInt(m_sensorReadingInterval))
        {
            m_valid = false;
        }
    }

    virtual ~DataCollectionStartMsg()
    {

    }

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const
    {
        Message::Serialise(serialiser);
        serialiser.serialise(m_sensorReadingInterval);
    }

    int getSensorReadingInterval() const { return m_sensorReadingInterval; }

private:
    int m_sensorReadingInterval;
};
