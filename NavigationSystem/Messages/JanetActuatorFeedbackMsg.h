#pragma once

#include "../MessageBus/Message.h"

class JanetActuatorFeedbackMsg : public Message {
   public:
    JanetActuatorFeedbackMsg(NodeID sourceID,
                             NodeID destinationID,
                             double wingsailFeedback,
                             double rudderFeedback,
                             double windvaneAngle,
                             double windvaneActuatorPos,
                             bool radioControllerOn)
        : Message(MessageType::JanetActuatorFeedback, sourceID, destinationID),
          m_WingsailFeedback(wingsailFeedback),
          m_RudderFeedback(rudderFeedback),
          m_WindvaneSelfSteeringAngle(windvaneAngle),
          m_WindvaneActuatorPosition(windvaneActuatorPos),
          m_RadioControllerOn(radioControllerOn) {}

    JanetActuatorFeedbackMsg(double wingsailFeedback,
                             double rudderFeedback,
                             double windvaneAngle,
                             double windvaneActuatorPos,
                             bool radioControllerOn)
        : Message(MessageType::JanetActuatorFeedback, NodeID::None, NodeID::None),
          m_WingsailFeedback(wingsailFeedback),
          m_RudderFeedback(rudderFeedback),
          m_WindvaneSelfSteeringAngle(windvaneAngle),
          m_WindvaneActuatorPosition(windvaneActuatorPos),
          m_RadioControllerOn(radioControllerOn) {}

    JanetActuatorFeedbackMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readDouble(m_RudderFeedback) ||
            !deserialiser.readDouble(m_WingsailFeedback)) {
            m_valid = false;
        }
    }

    virtual ~JanetActuatorFeedbackMsg() {}

    double wingsailFeedback() const { return m_WingsailFeedback; }
    double rudderFeedback() const { return m_RudderFeedback; }
    double windvaneSelfSteeringAngle() const { return m_WindvaneSelfSteeringAngle; }
    double windvaneActuatorPosition() const { return m_WindvaneActuatorPosition; }
    bool radioControllerOn() const { return m_RadioControllerOn; }

   private:
    double m_WingsailFeedback;  // degree in wing sail reference frame (clockwise from top view)
    double m_RudderFeedback;    // degree in vessel reference frame (clockwise from top view)
    double m_WindvaneSelfSteeringAngle;  // degree
    double m_WindvaneActuatorPosition;
    bool m_RadioControllerOn;
};
