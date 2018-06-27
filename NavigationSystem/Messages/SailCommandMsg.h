#pragma once

#include "../MessageBus/Message.h"

class SailCommandMsg : public Message {
   public:
    SailCommandMsg(NodeID sourceID, NodeID destinationID, float maxSailAngle)
        : Message(MessageType::SailCommand, sourceID, destinationID),
          m_MaxSailAngle(maxSailAngle) {}

    SailCommandMsg(float maxSailAngle)
        : Message(MessageType::SailCommand, NodeID::None, NodeID::None),
          m_MaxSailAngle(maxSailAngle) {}

    SailCommandMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readFloat(m_MaxSailAngle)) {
            m_valid = false;
        }
    }

    virtual ~SailCommandMsg() {}
    float maxSailAngle() const { return m_MaxSailAngle; }

    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);
        serialiser.serialise(m_MaxSailAngle);
    }

   private:
    float m_MaxSailAngle;  // degrees in boat reference frame. This value is always positive.
};
