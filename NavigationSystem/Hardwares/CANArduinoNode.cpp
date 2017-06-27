/*Handels incoming data from Ardunin */

#include "CANArduinoNode.h"
#include "Messages/ArduinoDataMsg.h"
#include "SystemServices/Timer.h"

#include <chrono>
#include <thread>

CANArduinoNode::CANArduinoNode(MessageBus& messageBus, CANService& canService, int time_filter_ms) : 
ActiveNode(NodeID::CANArduino, messageBus), CANFrameReceiver(canService, {701,702}), m_TimeBetweenMsgs(time_filter_ms)

{   
  m_RudderFeedback  = DATA_OUT_OF_RANGE;
  m_WingsailFeedback = DATA_OUT_OF_RANGE;
  m_WindvaneSelfSteerAngle = DATA_OUT_OF_RANGE;
	m_WindvaneActuatorPos = DATA_OUT_OF_RANGE;
	m_RC = DATA_OUT_OF_RANGE;
	}

CANArduinoNode::~CANArduinoNode(){

}

bool CANArduinoNode::init() {
	return true;
}

void CANArduinoNode::processMessage (const Message* message){

}

void CANArduinoNode::processFrame (CanMsg& msg) {
	if (msg.id == 701) {
		 m_RudderFeedback = (msg.data[1] << 8 | msg.data[0]);
     m_WingsailFeedback = (msg.data[3] << 8 | msg.data[2]);
     m_WindvaneSelfSteerAngle = (msg.data[5] << 8 | msg.data[4]);
     m_WindvaneActuatorPos = msg.data[7];

     //MessagePtr feedbackMsg = std::make_unique<ASPireActuatorFeedbackMsg>(rudderFeedback, wingsailFeedback,
     //                                                                            windvaneSelfSteerAngle, windvaneActuatorPos);
     //m_MsgBus.sendMessage(std::move(feedbackMsg))
		 
	} else if (msg.id == 702) {
		m_RC = (msg.data[1] << 8 | msg.data[0]);
		
		//MessagePtr arduinoMsg = std::make_unique<ArduinoDataMsg>(0,0,0,0,RC);
		//m_MsgBus.sendMessage(std::move(arduinoMsg));
	} 
}


void CANArduinoNode::start() {
	runThread(CANArduinoNodeThreadFunc);
	}

void CANArduinoNode::CANArduinoNodeThreadFunc(ActiveNode* nodePtr) {

 CANArduinoNode* node = dynamic_cast<CANArduinoNode*> (nodePtr);
		Timer timer;
		timer.start();

		while(true) {
			// Need to convert milliseconds into seconds for the argument
			timer.sleepUntil(node->m_TimeBetweenMsgs*1.0f / 1000);
			node->m_lock.lock();

			if( node->m_RudderFeedback == node->DATA_OUT_OF_RANGE &&  node->m_WindvaneSelfSteerAngle == node->DATA_OUT_OF_RANGE && node->m_WingsailFeedback == node->DATA_OUT_OF_RANGE){
				node->m_lock.unlock();
				continue;
			}

		MessagePtr feebackData = std::make_unique<ASPireActuatorFeedbackMsg>(node->m_RudderFeedback, node->m_WingsailFeedback, node->m_WindvaneSelfSteerAngle);
		node->m_MsgBus.sendMessage(std::move(feebackData));

		node->m_lock.unlock();

		timer.reset();
		}
}