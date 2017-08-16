#pragma once

#include "MessageBus/Message.h"


class WingSailCommandMsg : public Message {
public:
    WingSailCommand(NodeID sourceID, NodeID destinationID, double tailAngle)
    : Message(MessageType::WingSailCommand, sourceID, destinationID),
    m_TailAngle(tailAngle)
    {  }

    WingSailCommand(double tailAngle)
    : Message(MessageType::WingSailCommand, NodeID::None, NodeID::None),
      m_TailAngle(tailAngle)
    {  }

    virtual ~WingSailCommandMsg() { }
    double tailAngle() const { return m_TailAngle;}

private:
    double m_TailAngle;
};