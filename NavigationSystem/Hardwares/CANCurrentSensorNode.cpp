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
#include "../MessageBus/MessageTypes.h"
#include "CAN_Services/CanBusCommon/CanMessageHandler.h"

const float DATA_OUT_OF_RANGE = -2000; // as uint16_t, cannot use -2000.

CANCurrentSensorNode::CANCurrentSensorNode(MessageBus& messageBus, DBHandler& dbhandler, CANService& canService) :
ActiveNode(NodeID::CANCurrentSensor, messageBus), CANFrameReceiver(canService, {MSG_ID_CURRENT_SENSOR_DATA,MSG_ID_CURRENT_SENSOR_DATA_POWER_UNIT}), m_LoopTime (0.5), m_db(dbhandler)
{
        m_current = DATA_OUT_OF_RANGE;
        m_voltage = DATA_OUT_OF_RANGE;
        //m_element = "undefined";
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
    //Logger::info("Received current sensor readings from CanBus");
    Float16Compressor fltCompressor;
	CanMessageHandler messageHandler(msg);
	uint16_t comp_current, comp_voltage;
	uint8_t  sensor_id, rol_num;

	if (messageHandler.getMessageId() == MSG_ID_CURRENT_SENSOR_DATA)
    {
        messageHandler.getData( &comp_current, CURRENT_SENSOR_CURRENT_START, CURRENT_SENSOR_CURRENT_DATASIZE, CURRENT_SENSOR_CURRENT_IN_BYTE);
        messageHandler.getData( &comp_voltage, CURRENT_SENSOR_VOLTAGE_START, CURRENT_SENSOR_VOLTAGE_DATASIZE, CURRENT_SENSOR_VOLTAGE_IN_BYTE);
        messageHandler.getData( &sensor_id,    CURRENT_SENSOR_ID_START,      CURRENT_SENSOR_ID_DATASIZE,      CURRENT_SENSOR_ID_IN_BYTE);
        messageHandler.getData( &rol_num,      CURRENT_SENSOR_ROL_NUM_START, CURRENT_SENSOR_ROL_NUM_DATASIZE, CURRENT_SENSOR_ROL_NUM_IN_BYTE);

        m_current = fltCompressor.decompress(comp_current);
        m_voltage = fltCompressor.decompress(comp_voltage);
        
        m_element = static_cast<SensedElement>(sensor_id);        //TO TRY
        MessagePtr currentSensorDataMsg = std::make_unique<CurrentSensorDataMsg>(static_cast<float>(m_current),
     		           static_cast<float>(m_voltage), static_cast<SensedElement>(m_element));
        Logger::info("Current sensor data: Current: %lf , Voltage: %lf , Sensor: %d \n",m_current,m_voltage,m_element);
    }
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
//		if( not (node->m_voltage == DATA_OUT_OF_RANGE &&  node->m_current == DATA_OUT_OF_RANGE))
				// Skipped for now  && node->m_element == "undefined"))
//		{
			MessagePtr currentSensorData = std::make_unique<CurrentSensorDataMsg>( node->m_current, 
				node->m_voltage, node->m_element);
			node->m_MsgBus.sendMessage(std::move(currentSensorData));
//		}
		node->m_lock.unlock();

		timer.sleepUntil(node->m_LoopTime);
		timer.reset();
	}
}
