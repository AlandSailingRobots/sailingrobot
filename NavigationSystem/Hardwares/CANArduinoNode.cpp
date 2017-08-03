/****************************************************************************************
*
* File:
* 		CANArduinoNode.cpp
*
* Purpose:
*		 Process messages from the arduino in the Actuatorunit via the CAN-Service including ActuatorFeedback and RC status . 
*
* Developer Notes:
*		 The CAN frame id numbers that this node subscribes to are:
*			701, 702, (more to be added later?)
*
***************************************************************************************/

#include "CANArduinoNode.h"
#include "SystemServices/Logger.h"


CANArduinoNode::CANArduinoNode(MessageBus& messageBus, CANService& canService, int time_filter_ms) :
ActiveNode(NodeID::CANArduino, messageBus), CANFrameReceiver(canService, {701,702}), m_loopTime (time_filter_ms)
{
	m_RudderFeedback  = DATA_OUT_OF_RANGE;
	m_WingsailFeedback = DATA_OUT_OF_RANGE;
	m_WindvaneSelfSteerAngle = DATA_OUT_OF_RANGE;
	m_WindvaneActuatorPos = DATA_OUT_OF_RANGE;
	m_Radio_Controller_On = DATA_OUT_OF_RANGE;
}

CANArduinoNode::~CANArduinoNode(){

}

bool CANArduinoNode::init() {
	return true;
}

void CANArduinoNode::processMessage (const Message* message){

}

void CANArduinoNode::processFrame (CanMsg& msg) {
	uint16_t rawData;

	if (msg.id == 701) {
		rawData = (msg.data[1] << 8 | msg.data[0]);		 
		m_RudderFeedback = Utility::mapInterval (rawData, 0, INT16_SIZE, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE);

		rawData = (msg.data[3] << 8 | msg.data[2]);
		m_WingsailFeedback = Utility::mapInterval (rawData, 0, INT16_SIZE, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);

		m_WindvaneSelfSteerAngle = (msg.data[5] << 8 | msg.data[4]);
		m_WindvaneActuatorPos = msg.data[7];
	}
	else if (msg.id == 702) {
		m_Radio_Controller_On = (msg.data[1] << 8 | msg.data[0]);
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
			
		node->m_lock.lock();
		if( node->m_RudderFeedback == node->DATA_OUT_OF_RANGE &&  node->m_WindvaneSelfSteerAngle == node->DATA_OUT_OF_RANGE &&
															node->m_WingsailFeedback == node->DATA_OUT_OF_RANGE && node->m_WindvaneActuatorPos == node->DATA_OUT_OF_RANGE && node->m_Radio_Controller_On ==node->DATA_OUT_OF_RANGE){
			node->m_lock.unlock();
			continue;
		}
		MessagePtr feebackData = std::make_unique<ASPireActuatorFeedbackMsg>( node->m_WingsailFeedback, node->m_RudderFeedback,
																		node->m_WindvaneSelfSteerAngle, node->m_WindvaneActuatorPos);
		node->m_MsgBus.sendMessage(std::move(feebackData));
		
		MessagePtr statusMsg = std::make_unique<ArduinoDataMsg>(0,0,0,0,node->m_Radio_Controller_On);
		node->m_MsgBus.sendMessage(std::move(statusMsg));

		node->m_lock.unlock();

		// Need to convert milliseconds into seconds for the argument
		timer.sleepUntil(node->m_loopTime*1.0f / 1000);
		timer.reset();
	}
}




