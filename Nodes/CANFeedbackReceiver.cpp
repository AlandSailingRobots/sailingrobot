#include "CANFeedbackReceiver.h"

#include "Messages/ASPireActuatorFeedbackMessage.h"

CANFeedbackReceiver::CANFeedbackReceiver(MessageBus& messageBus, CANService& canService, uint32_t canID) : 
                    Node(NodeID::CANFeedbackReceiver, messageBus), CANFrameReceiver(canService, canID)

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

        MessagePtr feedbackMsg = std::make_unique<ASPireActuatorFeedbackMessage>(rudderFeedback, wingsailFeedback,
                                                                                 windvaneSelfSteerAngle, windvaneActuatorPos);

        m_MsgBus.sendMessage(std::move(feedbackMsg));
    }
}