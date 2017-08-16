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
      m_RudderAngle(rudderangle)
    {  }  


    virtual ~RudderCommandMsg() { }
    double rudderAngle() const { return m_RudderAngle;}


private:
    double m_RudderAngle;
};