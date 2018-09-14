/****************************************************************************************
 *
 * File:
 * 		ExternalControlMsg.h
 *
 * Purpose:
 *		Contains a boolean that will notify the Sailing Logic that an external control
 *      will send actuator messages instead
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"

class ExternalControlMsg : public Message {
   public:
    ExternalControlMsg(NodeID destinationID, NodeID sourceID, bool externalControlActive)
        : Message(MessageType::ExternalControl, sourceID, destinationID),
          m_externalControlActive(externalControlActive) {}

    ExternalControlMsg(bool externalControlActive)
        : Message(MessageType::ExternalControl, NodeID::None, NodeID::None),
          m_externalControlActive(externalControlActive) {}

    ExternalControlMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readBool(m_externalControlActive)) {
            m_valid = false;
        }
    }

    virtual ~ExternalControlMsg() {}

    bool externalControlActive() { return m_externalControlActive; }

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);

        serialiser.serialise(m_externalControlActive);
    }

   private:
    bool m_externalControlActive;
};