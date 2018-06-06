/****************************************************************************************
*
* File:
* 		CANCurrentSensorNode.cpp
*
* Purpose:
*		 Process messages from the current sensors via the CAN Services .
*
* Developer Notes:
*		 The CAN frame id numbers that this node subscribes to are:
*			600
*
***************************************************************************************/

#include "CANCurrentSensorNode.h"
#include "CAN_Services/CanBusCommon/CanUtility.h"
#include "MessageBus/MessageTypes.h"
#include "CAN_Services/CanBusCommon/CanMessageHandler.h"

const int DATA_OUT_OF_RANGE = -2000;

CANCurrentSensorNode::CANCurrentSensorNode(MessageBus& messageBus, DBHandler& dbhandler, CANService& canService) :
ActiveNode(NodeID::CANCurrentSensor, messageBus), CANFrameReceiver(canService, {600}), m_LoopTime (0.5), m_db(dbhandler)
{
        m_current = DATA_OUT_OF_RANGE;
        m_voltage = DATA_OUT_OF_RANGE;
        m_element = "undefined";
	messageBus.registerNode(*this, MessageType::ServerConfigsReceived);

}

CANCurrentSensorNode::~CANCurrentSensorNode(){

}

bool CANCurrentSensorNode::init() {
	updateConfigsFromDB();
	return true;
}

void CANCurrentSensorNode::updateConfigsFromDB()
{
    //m_LoopTime = m_db.retrieveCellAsDouble("config_can_arduino","1","loop_time");
}

void CANCurrentSensorNode::processMessage (const Message* message){
	if(message->messageType() == MessageType::ServerConfigsReceived)
	{
		updateConfigsFromDB();
	}
}

void CANCurrentSensorNode::processFrame (CanMsg& msg) {
        Logger::info("Received marine sensor readings from CanBus");

	CanMessageHandler messageHandler(msg);

	if (messageHandler.getMessageId() == MSG_ID_CURRENT_SENSOR_DATA) {
                // Use get data instead(int)? Parse data here or add the routine in another file?
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
        MessagePtr currentSensorDataMsg = std::make_unique<CurrentSensorDataMsg>(static_cast<uint16_t>(current),
                                                      static_cast<uint16_t>(voltage), static_cast<uint8_t>(element));
}


void CANCurrentSensorNode::start() {
	runThread(CANCurrentSensorNodeThreadFunc);
}

void CANCurrentSensorNode::CANCurrentSensorNodeThreadFunc(ActiveNode* nodePtr) {


	CANCurrentSensorNode* node = dynamic_cast<CANCurrentSensorNode*> (nodePtr);
	Timer timer;
	timer.start();

	while(true) {

		node->m_lock.lock();
                // Don´t forget modifs here
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
