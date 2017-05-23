#pragma once

#include "Messages/Message.h"
#include "Messages/MessageTypes.h"
#include "Nodes/NodeIDs.h"

class ASPireActuatorFeedbackMessage : public Message {
public:
    ASPireActuatorFeedbackMessage(NodeID sourceID, NodeID destinationID, double wingsailFeedback,
                                double rudderFeedback, double windvaneAngle, double windvaneActuatorPos)
    : Message(MessageType::ActuatorControlASPire, sourceID, destinationID),
    m_WingsailFeedback(wingsailFeedback), m_RudderFeedback(rudderFeedback),
    m_WindvaneSelfSteeringAngle(windvaneAngle), m_WindvaneActuatorPosition(windvaneActuatorPos)
    {  }

    ASPireActuatorFeedbackMessage( double wingsailFeedback, double rudderFeedback, 
                                   double windvaneAngle, double windvaneActuatorPos)
    : Message(MessageType::ActuatorControlASPire, NodeID::None, NodeID::None),
    m_WingsailFeedback(wingsailFeedback), m_RudderFeedback(rudderFeedback),
    m_WindvaneSelfSteeringAngle(windvaneAngle), m_WindvaneActuatorPosition(windvaneActuatorPos)
    {  }

    virtual ~ASPireActuatorFeedbackMessage() { }

    double wingsailFeedback() const          { return m_WingsailFeedback; }
    double rudderFeedback() const            { return m_RudderFeedback; }
    double windvaneSelfSteeringAngle() const { return m_WindvaneSelfSteeringAngle; }
    double windvaneActuatorPosition()  const { return m_WindvaneActuatorPosition; }

private:
    double m_WingsailFeedback;
    double m_RudderFeedback;
    double m_WindvaneSelfSteeringAngle;
    double m_WindvaneActuatorPosition;
};