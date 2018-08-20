#ifndef SAILINGROBOT_VELVETACTUATORFEEDBACKMSG_H
#define SAILINGROBOT_VELVETACTUATORFEEDBACKMSG_H

#include "../MessageBus/Message.h"

class VelvetActuatorFeedbackMsg : public Message {
public:
    VelvetActuatorFeedbackMsg(NodeID sourceID,
                             NodeID destinationID,
                             double sailFeedback,
                             double rudderFeedback,
                             bool radioControllerOn)
            : Message(MessageType::VelvetActuatorFeedback, sourceID, destinationID),
              m_SailFeedback(sailFeedback),
              m_RudderFeedback(rudderFeedback),
              m_RadioControllerOn(radioControllerOn) {}

    VelvetActuatorFeedbackMsg(double sailFeedback,
                             double rudderFeedback,
                             bool radioControllerOn)
            : Message(MessageType::VelvetActuatorFeedback, NodeID::None, NodeID::None),
              m_SailFeedback(sailFeedback),
              m_RudderFeedback(rudderFeedback),
              m_RadioControllerOn(radioControllerOn) {}

    VelvetActuatorFeedbackMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readDouble(m_RudderFeedback) ||
            !deserialiser.readDouble(m_SailFeedback)) {
            m_valid = false;
        }
    }

    virtual ~VelvetActuatorFeedbackMsg() {}

    double sailFeedback() const { return m_SailFeedback; }
    double rudderFeedback() const { return m_RudderFeedback; }
    bool radioControllerOn() const { return m_RadioControllerOn; }

private:
    double m_SailFeedback;  // degree in wing sail reference frame (clockwise from top view)
    double m_RudderFeedback;    // degree in vessel reference frame (clockwise from top view)
    bool m_RadioControllerOn;
};


#endif //SAILINGROBOT_VELVETACTUATORFEEDBACKMSG_H
