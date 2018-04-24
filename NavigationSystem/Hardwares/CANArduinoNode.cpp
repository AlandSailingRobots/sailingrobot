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
#include "CAN_Services/CanBusCommon/CanUtility.h"
#include "MessageBus/MessageTypes.h"
#include "CAN_Services/CanBusCommon/CanMessageHandler.h"

const int DATA_OUT_OF_RANGE = -2000;

CANArduinoNode::CANArduinoNode(MessageBus& messageBus, DBHandler& dbhandler, CANService& canService) :
ActiveNode(NodeID::CANArduino, messageBus), CANFrameReceiver(canService, {701,702}), m_LoopTime (0.5), m_db(dbhandler)
{
	m_RudderFeedback  = DATA_OUT_OF_RANGE;
	m_WingsailFeedback = DATA_OUT_OF_RANGE;
	m_WindvaneSelfSteerAngle = DATA_OUT_OF_RANGE;
	m_WindvaneActuatorPos = DATA_OUT_OF_RANGE;
	m_Radio_Controller_On = DATA_OUT_OF_RANGE;
	messageBus.registerNode(*this, MessageType::ServerConfigsReceived);

}

CANArduinoNode::~CANArduinoNode(){

}

bool CANArduinoNode::init() {
	updateConfigsFromDB();
	return true;
}

void CANArduinoNode::updateConfigsFromDB()
{
    m_LoopTime = m_db.retrieveCellAsDouble("config_can_arduino","1","loop_time");
}

void CANArduinoNode::processMessage (const Message* message){
	if(message->messageType() == MessageType::ServerConfigsReceived)
	{
		updateConfigsFromDB();
	}
}

void CANArduinoNode::processFrame (CanMsg& msg) {

	CanMessageHandler messageHandler(msg);

	if (messageHandler.getMessageId() == MSG_ID_AU_FEEDBACK) {
		m_RudderFeedback = static_cast<float>(messageHandler.getMappedData(
				RUDDER_ANGLE_DATASIZE, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE));

		m_WingsailFeedback = static_cast<float>(messageHandler.getMappedData(
				WINGSAIL_ANGLE_DATASIZE, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE));

		m_WindvaneSelfSteerAngle = static_cast<float>(messageHandler.getMappedData(
				WINDVANE_SELFSTEERING_DATASIZE, WINDVANE_SELFSTEERING_ANGLE_MIN, WINDVANE_SELFSTEERING_ANGLE_MAX));

		m_WindvaneActuatorPos = messageHandler.getData(WINDVANE_ACTUATOR_POSITION_DATASIZE);
	}
	else if (messageHandler.getMessageId() == MSG_ID_RC_STATUS) {
		m_Radio_Controller_On = messageHandler.getData(RADIOCONTROLLER_ON_DATASIZE);
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

		if( not (node->m_RudderFeedback == DATA_OUT_OF_RANGE &&  node->m_WindvaneSelfSteerAngle == DATA_OUT_OF_RANGE &&
				node->m_WingsailFeedback == DATA_OUT_OF_RANGE && node->m_WindvaneActuatorPos == DATA_OUT_OF_RANGE))
		{
			MessagePtr feebackData = std::make_unique<ASPireActuatorFeedbackMsg>( node->m_WingsailFeedback, 
				node->m_RudderFeedback, node->m_WindvaneSelfSteerAngle, node->m_WindvaneActuatorPos, node->m_Radio_Controller_On);
			node->m_MsgBus.sendMessage(std::move(feebackData));
		}
		node->m_lock.unlock();

		timer.sleepUntil(node->m_LoopTime);
		timer.reset();
	}
}
