/****************************************************************************************
 *
 * File:
 * 		MessageVerifier.h
 *
 * Purpose:
 *		Stores messages, and verifies that other messages have the same data.
 *
 *
 *
 ***************************************************************************************/
#pragma once

#include "../MessageBus/Message.h"
#include "../MessageBus/Node.h"
#include "../Messages/ASPireActuatorFeedbackMsg.h"
#include "../Messages/WindStateMsg.h"

class MessageVerifier : public Node {
   public:
    MessageVerifier(MessageBus& msgBus) : Node(NodeID::MessageVerifier, msgBus) {
        msgBus.registerNode(*this);
        msgBus.registerNode(*this, MessageType::WindState);
        msgBus.registerNode(*this, MessageType::ASPireActuatorFeedback);
    }

    virtual ~MessageVerifier() {}

    bool init() { return true; }

    void processMessage(const Message* message) {
        MessageType type = message->messageType();

        // I guess the const shouldn't be removed in practice

        if (type == MessageType::WindState) {
            m_WindStateMsg = dynamic_cast<WindStateMsg*>(const_cast<Message*>(message));
        } else if (type == MessageType::ASPireActuatorFeedback) {
            m_FeedbackMsg = dynamic_cast<ASPireActuatorFeedbackMsg*>(const_cast<Message*>(message));
        }
    }

    bool verifyWindStateMsg(const WindStateMsg* otherMsg) {
        if (m_WindStateMsg == NULL) {
            return false;
        }

        if (m_WindStateMsg->trueWindSpeed() == otherMsg->trueWindSpeed() &&
            m_WindStateMsg->trueWindDirection() == otherMsg->trueWindDirection() &&
            m_WindStateMsg->apparentWindSpeed() == otherMsg->apparentWindSpeed() &&
            m_WindStateMsg->apparentWindDirection() == otherMsg->apparentWindDirection()) {
            return true;
        }

        else {
            m_MsgBus.sendMessage(std::make_unique<WindStateMsg>(
                otherMsg->trueWindSpeed(), otherMsg->trueWindDirection(),
                otherMsg->apparentWindSpeed(), otherMsg->apparentWindDirection()));
        }

        return false;
    }

    bool verifyActuatorFeedbackMsg(const ASPireActuatorFeedbackMsg* otherMsg) {
        if (m_FeedbackMsg == NULL) {
            return false;
        }

        if (m_FeedbackMsg->rudderFeedback() == otherMsg->rudderFeedback() &&
            m_FeedbackMsg->wingsailFeedback() == otherMsg->wingsailFeedback() &&
            m_FeedbackMsg->windvaneSelfSteeringAngle() == otherMsg->windvaneSelfSteeringAngle() &&
            m_FeedbackMsg->windvaneActuatorPosition() == otherMsg->windvaneActuatorPosition()) {
            return true;
        }

        return false;
    }

   private:
    WindStateMsg* m_WindStateMsg;
    ASPireActuatorFeedbackMsg* m_FeedbackMsg;
};
