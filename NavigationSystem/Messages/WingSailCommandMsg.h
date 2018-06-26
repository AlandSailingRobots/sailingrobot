#pragma once

#include "../MessageBus/Message.h"

class WingSailCommandMsg : public Message {
   public:
    WingSailCommandMsg(NodeID sourceID, NodeID destinationID, float tailAngle)
        : Message(MessageType::WingSailCommand, sourceID, destinationID), m_TailAngle(tailAngle) {}

    WingSailCommandMsg(float tailAngle)
        : Message(MessageType::WingSailCommand, NodeID::None, NodeID::None),
          m_TailAngle(tailAngle) {}

    WingSailCommandMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readFloat(m_TailAngle)) {
            m_valid = false;
        }
    }

    virtual ~WingSailCommandMsg() {}
    float tailAngle() const { return m_TailAngle; }

    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);
        serialiser.serialise(m_TailAngle);
    }

   private:
    float m_TailAngle;  // degree [-26, +26[ in wing sail reference frame (clockwise from top view)
};
