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
#include "../MessageBus/MessageTypes.h"
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
    m_LoopTime = m_db.tableColumnDouble("config_can_arduino", "loop_time", "1");
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
		messageHandler.getMappedData(&m_RudderFeedback,
				RUDDER_ANGLE_DATASIZE, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE);

		messageHandler.getMappedData(&m_WingsailFeedback,
				WINGSAIL_ANGLE_DATASIZE, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);


		messageHandler.getMappedData(&m_WindvaneSelfSteerAngle,
				WINDVANE_SELFSTEERING_DATASIZE, WINDVANE_SELFSTEERING_ANGLE_MIN, WINDVANE_SELFSTEERING_ANGLE_MAX);

		messageHandler.getData(&m_WindvaneActuatorPos, WINDVANE_ACTUATOR_POSITION_DATASIZE);
	}
	else if (messageHandler.getMessageId() == MSG_ID_RC_STATUS) {
		messageHandler.getData(&m_Radio_Controller_On, RADIOCONTROLLER_ON_DATASIZE);
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
