#pragma once

#include "MessageBus/Message.h"


class ASPireActuatorFeedbackMsg : public Message {
public:
    ASPireActuatorFeedbackMsg(NodeID sourceID, NodeID destinationID, double wingsailFeedback,
                                double rudderFeedback, double windvaneAngle, double windvaneActuatorPos, bool radioControllerOn)
    : Message(MessageType::ASPireActuatorFeedback, sourceID, destinationID),
    m_WingsailFeedback(wingsailFeedback), m_RudderFeedback(rudderFeedback),
    m_WindvaneSelfSteeringAngle(windvaneAngle), m_WindvaneActuatorPosition(windvaneActuatorPos), m_RadioControllerOn(radioControllerOn)
    {  }

    ASPireActuatorFeedbackMsg( double wingsailFeedback, double rudderFeedback,
                                   double windvaneAngle, double windvaneActuatorPos, bool radioControllerOn)
    : Message(MessageType::ASPireActuatorFeedback, NodeID::None, NodeID::None),
    m_WingsailFeedback(wingsailFeedback), m_RudderFeedback(rudderFeedback),
    m_WindvaneSelfSteeringAngle(windvaneAngle), m_WindvaneActuatorPosition(windvaneActuatorPos), m_RadioControllerOn(radioControllerOn)
    {  }

    virtual ~ASPireActuatorFeedbackMsg() { }

    double wingsailFeedback() const          { return m_WingsailFeedback; }
    double rudderFeedback() const            { return m_RudderFeedback; }
    double windvaneSelfSteeringAngle() const { return m_WindvaneSelfSteeringAngle; }
    double windvaneActuatorPosition()  const { return m_WindvaneActuatorPosition; }
	bool radioControllerOn() 				   const { return m_RadioControllerOn; }

private:
    double m_WingsailFeedback;
    double m_RudderFeedback;
    double m_WindvaneSelfSteeringAngle;
    double m_WindvaneActuatorPosition;
	bool   m_RadioControllerOn;
};
