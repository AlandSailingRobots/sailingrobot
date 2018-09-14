//
//	Wind State Message containing True Wind Speed & Direction,
//   as well as Apparent Wind Speed & Direction
//

#pragma once

#include "MessageBus/Message.h"

class WindStateMsg : public Message {
   public:
    WindStateMsg(NodeID sourceID,
                 NodeID destinationID,
                 double trueWindSpeed,
                 double trueWindDir,
                 double ApparentWindSpeed,
                 double ApparentWindDir)
        : Message(MessageType::WindState, sourceID, destinationID),
          m_trueWindSpeed(trueWindSpeed),
          m_trueWindDir(trueWindDir),
          m_apparentWindSpeed(ApparentWindSpeed),
          m_apparentWindDir(ApparentWindDir) {}

    WindStateMsg(double trueWindSpeed,
                 double trueWindDir,
                 double ApparentWindSpeed,
                 double ApparentWindDir)
        : Message(MessageType::WindState, NodeID::None, NodeID::None),
          m_trueWindSpeed(trueWindSpeed),
          m_trueWindDir(trueWindDir),
          m_apparentWindSpeed(ApparentWindSpeed),
          m_apparentWindDir(ApparentWindDir) {}

    WindStateMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readDouble(m_trueWindSpeed) || !deserialiser.readDouble(m_trueWindDir) ||
            !deserialiser.readDouble(m_apparentWindSpeed) ||
            !deserialiser.readDouble(m_apparentWindDir)) {
            m_valid = false;
        }
    }

    virtual ~WindStateMsg() {}

    double trueWindSpeed() const { return m_trueWindSpeed; }
    double trueWindDirection() const { return m_trueWindDir; }
    double apparentWindSpeed() const { return m_apparentWindSpeed; }
    double apparentWindDirection() const { return m_apparentWindDir; }

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);

        serialiser.serialise(m_trueWindSpeed);
        serialiser.serialise(m_trueWindDir);
        serialiser.serialise(m_apparentWindSpeed);
        serialiser.serialise(m_apparentWindDir);
    }

   private:
    double m_trueWindSpeed;      // m/s
    double m_trueWindDir;        // degree [0, 360[ in North-East reference frame (clockwise)
    double m_apparentWindSpeed;  // m/s
    double m_apparentWindDir;    // degree [0, 360[ in vessel reference frame (clockwise)
};