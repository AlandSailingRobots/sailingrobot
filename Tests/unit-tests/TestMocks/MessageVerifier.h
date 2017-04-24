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

#include "Nodes/Node.h"
#include "Messages/Message.h"
#include "Messages/WindStateMsg.h"



class MessageVerifier : public Node {
public:
    MessageVerifier(MessageBus& msgBus) : Node(NodeID::MessageVerifier, msgBus)
    { 
        msgBus.registerNode(*this, MessageType::WindState);
    }

    virtual ~MessageVerifier() {}

    bool init() { return true; }

    void processMessage(const Message* message){
        MessageType type = message->messageType();

        // I guess the const shouldn't be removed in practice

        if(type == MessageType::WindState){
            m_WindStateMsg = dynamic_cast<WindStateMsg*>(const_cast<Message*>(message));
        }

    }

    bool verifyWindStateMsg(const WindStateMsg* otherMsg){
        if(m_WindStateMsg == NULL){
            return false;
        }

        // Overload operator in WindStateMsg ??

        return(m_WindStateMsg->trueWindSpeed() == otherMsg->trueWindSpeed() &&
               m_WindStateMsg->trueWindDirection() == otherMsg->trueWindDirection() &&
               m_WindStateMsg->apparentWindSpeed() == otherMsg->apparentWindSpeed() &&
               m_WindStateMsg->apparentWindDirection() == otherMsg->apparentWindDirection());

    }


private:
    WindStateMsg* m_WindStateMsg;

};