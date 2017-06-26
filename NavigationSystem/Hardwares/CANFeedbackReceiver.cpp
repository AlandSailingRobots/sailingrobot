#include "CANFeedbackReceiver.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "SystemServices/Timer.h"

#include <chrono>
#include <thread>

CANFeedbackReceiver::CANFeedbackReceiver(MessageBus& messageBus, CANService& canService, int time_filter_ms) : 
                    Node(NodeID::CANFeedbackReceiver, messageBus), CANFrameReceiver(canService, 701)

{  

}


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
/*
void CANFeedbackReceiver::CANFeedbackReceiverThreadFunc (ActiveNode* nodePtr) {
   
	CANFeedbackReceiver* node = dynamic_cast<CANFeedbackReceiver*> (nodePtr);
	Timer timer;
	timer.start();
	while(true) {
		timer.sleepUntil(node->m_TimeBetweenMsgs*1.0f / 1000);
		node->m_lock.lock();
		MessagePtr feedback = std::make_uni
		
*/
