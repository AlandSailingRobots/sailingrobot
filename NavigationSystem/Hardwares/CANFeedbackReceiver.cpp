#include "CANFeedbackReceiver.h"

#include "Messages/ASPireActuatorFeedbackMsg.h"

CANFeedbackReceiver::CANFeedbackReceiver(MessageBus& messageBus, CANService& canService) : 
                    Node(NodeID::CANFeedbackReceiver, messageBus), CANFrameReceiver(canService, 701)

{  }


CANFeedbackReceiver::~CANFeedbackReceiver() {

}


bool CANFeedbackReceiver::init() {
    return true;
}

void CANFeedbackReceiver::processMessage(const Message* message) {

}

void CANFeedbackReceiver::processFrame(CanMsg& msg) {
    if(msg.id == 701) {
        uint16_t rudderFeedback = (msg.data[1] << 8 | msg.data[0]);
        uint16_t wingsailFeedback = (msg.data[3] << 8 | msg.data[2]);
        uint16_t windvaneSelfSteerAngle = (msg.data[5] << 8 | msg.data[4]);
        uint8_t windvaneActuatorPos = msg.data[7];

        MessagePtr feedbackMsg = std::make_unique<ASPireActuatorFeedbackMsg>(rudderFeedback, wingsailFeedback,
                                                                                 windvaneSelfSteerAngle, windvaneActuatorPos);

        m_MsgBus.sendMessage(std::move(feedbackMsg));
    }
}