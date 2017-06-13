


#pragma once

#include "MessageBus/Message.h"


class ActuatorControlASPireMessage : public Message {
public:
    ActuatorControlASPireMessage(NodeID sourceID, NodeID destinationID, double wingsailServoAngle,
                                double rudderangle, bool selfSteering)
    : Message(MessageType::ActuatorControlASPire, sourceID, destinationID),
      m_WingsailServoAngle(wingsailServoAngle), m_RudderAngle(rudderangle),
      m_WindvaneSelfSteeringOn(selfSteering)
    {  }

    ActuatorControlASPireMessage(double wingsailServoAngle, double rudderangle,
                                  bool selfSteering)
    : Message(MessageType::ActuatorControlASPire, NodeID::None, NodeID::None),
      m_WingsailServoAngle(wingsailServoAngle), m_RudderAngle(rudderangle),
      m_WindvaneSelfSteeringOn(selfSteering)
    {  }  

    virtual ~ActuatorControlASPireMessage() { }

    double wingsailServoAngle() const { return m_WingsailServoAngle; }
    double rudderAngle() const        { return m_RudderAngle; }
    bool windvaneSelfSteering() const { return m_WindvaneSelfSteeringOn; }
private:
    double m_WingsailServoAngle;
    double m_RudderAngle;
    bool m_WindvaneSelfSteeringOn;
};