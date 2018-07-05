/****************************************************************************************
 *
 * File:
 * 		LidarMsg
 *
 * Purpose:
 *		Contains a distance in cm
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "../MessageBus/Message.h"

class LidarMsg : public Message {
   public:
    LidarMsg(NodeID destinationID, NodeID sourceID, int distance)

        : Message(MessageType::LidarData, sourceID, destinationID), m_distance(distance) {}

    LidarMsg(NodeID sourceID, int distance)

        : Message(MessageType::LidarData, sourceID, NodeID::None), m_distance(distance) {}

    LidarMsg(int distance)

        : Message(MessageType::LidarData, NodeID::None, NodeID::None), m_distance(distance) {}

    LidarMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readInt(m_distance)) {
            m_valid = false;
        }
    }

    virtual ~LidarMsg() {}

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);

        serialiser.serialise(m_distance);
    }

    int distance() { return m_distance; }

   private:
    int m_distance;
};
