#pragma once

#include "MessageBus/Message.h"


class RudderCommandMsg : public Message {
public:
 

    RudderCommandMsg(NodeID sourceID, NodeID destinationID, double rudderAngle)
    : Message(MessageType::RudderCommand, sourceID, destinationID),
    m_RudderAngle(rudderAngle)
    {  }

    RudderCommandMsg(double rudderAngle)
    : Message(MessageType::RudderCommand, NodeID::None, NodeID::None),
      m_RudderAngle(rudderAngle)
    {  }  


    virtual void Serialise(MessageSerialiser& serialiser) const
    {
        Message::Serialise(serialiser);

        serialiser.serialise(m_RudderAngle);
    }

    virtual ~RudderCommandMsg() { }
    double rudderAngle() const { return m_RudderAngle;}


private:
    double m_RudderAngle;
};