#pragma once

#include "MessageBus/Message.h"


class WingSailCommandMsg : public Message {
public:
    WingSailCommandMsg(NodeID sourceID, NodeID destinationID, double tailAngle)
        :Message(MessageType::WingSailCommand, sourceID, destinationID), m_TailAngle(tailAngle)
    {  }

    WingSailCommandMsg(double tailAngle)
        :Message(MessageType::WingSailCommand, NodeID::None, NodeID::None), m_TailAngle(tailAngle)
    {  }

    WingSailCommandMsg(MessageDeserialiser deserialiser)
        :Message(deserialiser)
    {
        if( !deserialiser.readDouble(m_TailAngle))
        {
            m_valid = false;
        }
    }

    virtual ~WingSailCommandMsg() { }
    double tailAngle() const { return m_TailAngle;}

    virtual void Serialise(MessageSerialiser& serialiser) const
    {
        Message::Serialise(serialiser);

        serialiser.serialise(m_TailAngle);
    }

private:
    double m_TailAngle;
};