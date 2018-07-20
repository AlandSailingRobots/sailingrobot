/****************************************************************************************
*
* File:
* 		CANArduinoNode.cpp
*
* Purpose:
*		 Sends data to the Actuator unit with the rudder and wingsail angles over the CAN bus. 
*
* Developer Notes:
*		 The CAN frame ID number that this node subscribe to are:
*			700
*
***************************************************************************************/
#include "ActuatorNodeASPire.h"
#include "CAN_Services/CanBusCommon/CanUtility.h"
#include "CAN_Services/CanBusCommon/CanMessageHandler.h"


ActuatorNodeASPire::ActuatorNodeASPire(MessageBus& msgBus, CANService& CANService)
: Node(NodeID::ActuatorNodeASPire, msgBus), m_CANService(&CANService), m_rudderAngle(0),
	m_wingsailAngle(0), m_windvaneSelfSteeringOn(0)
{
	msgBus.registerNode(*this, MessageType::WingSailCommand);
	msgBus.registerNode(*this, MessageType::RudderCommand);
}

ActuatorNodeASPire::~ActuatorNodeASPire()
{

}

bool ActuatorNodeASPire::init()
{
	return true;
}

void ActuatorNodeASPire::processMessage(const Message* message)
{    
	MessageType type = message->messageType();

	if(type == MessageType::WingSailCommand)
	{
    	const WingSailCommandMsg* actMsg = dynamic_cast<const WingSailCommandMsg*>(message);
		  m_wingsailAngle = actMsg->tailAngle();
    }
    else if (type == MessageType::RudderCommand)
    {
    	const RudderCommandMsg* actMsg = dynamic_cast<const RudderCommandMsg*>(message);
		m_rudderAngle = actMsg->rudderAngle();
    }

    sendCommandMessage();
}

void ActuatorNodeASPire::sendCommandMessage()
{
	CanMessageHandler messageHandler(MSG_ID_AU_CONTROL);

	messageHandler.encodeMappedMessage(RUDDER_ANGLE_DATASIZE, m_rudderAngle, MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
	messageHandler.encodeMappedMessage(WINGSAIL_ANGLE_DATASIZE, m_wingsailAngle, MIN_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);
	messageHandler.encodeMessage(WINDVANE_SELFSTEERING_ON_DATASIZE, m_windvaneSelfSteeringOn);

	// New version, have to do the changes on the arduino side before enabling
//	messageHandler.encodeMappedMessage(m_rudderAngle, RUDDER_ANGLE_START, RUDDER_ANGLE_DATASIZE, RUDDER_ANGLE_IN_BYTE, MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
//	messageHandler.encodeMappedMessage(m_wingsailAngle, WINGSAIL_ANGLE_START,  WINGSAIL_ANGLE_DATASIZE, WINGSAIL_ANGLE_IN_BYTE,  MIN_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);
//  messageHandler.encodeMessage(m_windvaneSelfSteeringOn, WINDVANE_SELFSTEERING_ON_START, WINDVANE_SELFSTEERING_ON_DATASIZE, WINDVANE_SELFSTEERING_ON_IN_BYTE);
//	messageHandler.bitsetToCanMsg();


	CanMsg Cmsg = messageHandler.getMessage();
	m_CANService->sendCANMessage(Cmsg);
}

