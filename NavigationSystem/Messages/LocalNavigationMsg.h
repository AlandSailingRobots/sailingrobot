#pragma once

#include "MessageBus/Message.h"


class LocalNavigationMsg : public Message {

public:

    LocalNavigationMsg(NodeID sourceID, NodeID destinationID, float targetCourse, float targetSpeed, bool beatingState, bool targetTackStarboard)
        :Message(MessageType::LocalNavigation, sourceID, destinationID), m_TargetCourse(targetCourse), m_TargetSpeed(targetSpeed),
        m_BeatingState(beatingState), m_TargetTackStarboard(targetTackStarboard)
    { }

    LocalNavigationMsg(float targetCourse, float targetSpeed, bool beatingState, bool targetTackStarboard)
        :Message(MessageType::LocalNavigation, NodeID::None, NodeID::None), m_TargetCourse(targetCourse), m_TargetSpeed(targetSpeed),
        m_BeatingState(beatingState), m_TargetTackStarboard(targetTackStarboard)
    { }

    LocalNavigationMsg(MessageDeserialiser deserialiser)
    : Message(deserialiser)
    {
        if(!deserialiser.readFloat(m_TargetCourse) ||
           !deserialiser.readFloat(m_TargetSpeed) ||
           !deserialiser.readBool(m_BeatingState) ||
           !deserialiser.readBool(m_TargetTackStarboard))
        {
            m_valid = false;
        }
    }

    virtual ~LocalNavigationMsg() { }


    float targetCourse() const { return m_TargetCourse; }
    float targetSpeed() const { return m_TargetSpeed; }
    bool beatingState() const { return m_BeatingState; }
    bool targetTackStarboard() const { return m_TargetTackStarboard; }


    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const
    {
        Message::Serialise(serialiser);

        serialiser.serialise(m_TargetCourse);
        serialiser.serialise(m_TargetSpeed);
        serialiser.serialise(m_BeatingState);
        serialiser.serialise(m_TargetTackStarboard);
    }

private:

    float   m_TargetCourse;         // degree [0, 360[ in North-East reference frame (clockwise)
    float   m_TargetSpeed;          // m/s
    bool    m_BeatingState;         // True if the vessel is in beating motion (zig-zag motion).
    bool    m_TargetTackStarboard;  // True if the desired tack of the vessel is starboard.
};
