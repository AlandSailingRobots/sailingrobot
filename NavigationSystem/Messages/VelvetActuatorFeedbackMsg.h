#ifndef SAILINGROBOT_VELVETACTUATORFEEDBACKMSG_H
#define SAILINGROBOT_VELVETACTUATORFEEDBACKMSG_H

#include "../MessageBus/Message.h"

class VelvetActuatorFeedbackMsg : public Message {
public:
    VelvetActuatorFeedbackMsg(NodeID sourceID,
                             NodeID destinationID,
                             double wingsailFeedback,
                             double rudderFeedback,
                             bool radioControllerOn)
            : Message(MessageType::VelvetActuatorFeedback, sourceID, destinationID),
              m_WingsailFeedback(wingsailFeedback),
              m_RudderFeedback(rudderFeedback),
              m_RadioControllerOn(radioControllerOn) {}

    VelvetActuatorFeedbackMsg(double wingsailFeedback,
                             double rudderFeedback,
                             bool radioControllerOn)
            : Message(MessageType::VelvetActuatorFeedback, NodeID::None, NodeID::None),
              m_WingsailFeedback(wingsailFeedback),
              m_RudderFeedback(rudderFeedback),
              m_RadioControllerOn(radioControllerOn) {}

    VelvetActuatorFeedbackMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        if (!deserialiser.readDouble(m_RudderFeedback) ||
            !deserialiser.readDouble(m_WingsailFeedback)) {
            m_valid = false;
        }
    }

    virtual ~VelvetActuatorFeedbackMsg() {}

    double wingsailFeedback() const { return m_WingsailFeedback; }
    double rudderFeedback() const { return m_RudderFeedback; }
    bool radioControllerOn() const { return m_RadioControllerOn; }

private:
    double m_WingsailFeedback;  // degree in wing sail reference frame (clockwise from top view)
    double m_RudderFeedback;    // degree in vessel reference frame (clockwise from top view)
    bool m_RadioControllerOn;
};


#endif //SAILINGROBOT_VELVETACTUATORFEEDBACKMSG_H
