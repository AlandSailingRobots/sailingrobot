/****************************************************************************************
*
* File:
* 		CANArduinoNode.cpp
*
* Purpose:
*		 Sends data to the Actuator unit with the rudder and wingsail angles over the CAN bus. 
*
* Developer Notes:
*		 The CAN frame ID nubmer that this node subscribe to are:
*			700
*
***************************************************************************************/

#include "Hardwares/ActuatorNodeASPire.h"


ActuatorNodeASPire::ActuatorNodeASPire(MessageBus& msgBus, CANService& CANService)
: Node(NodeID::ActuatorNodeASPire, msgBus), m_CANService(&CANService)
{
	msgBus.registerNode(*this, MessageType::ActuatorControlASPire);
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

	if(type == MessageType::ActuatorControlASPire) {
		const ActuatorControlASPireMessage* actMsg = dynamic_cast<const ActuatorControlASPireMessage*>(message);
		m_rudderAngle = actMsg->rudderAngle();
		m_wingsailAngle = actMsg->wingsailServoAngle();
		m_windvaneSelfSteeringOn = actMsg->windvaneSelfSteering();
					  
		uint16_t rudderAngle16 = Utility::mapInterval (m_rudderAngle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 0 , INT16_SIZE);
		uint16_t wingsailAngle16 = Utility::mapInterval (m_wingsailAngle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE, 0 , INT16_SIZE);
		
		CanMsg Cmsg;
		Cmsg.id = 700;
		Cmsg.header.ide = 0;
		Cmsg.header.length = 8;

		(Cmsg.data[0] = rudderAngle16 & 0xff);
		(Cmsg.data[1] = rudderAngle16 >> 8);
		(Cmsg.data[2] = wingsailAngle16 & 0xff);
		(Cmsg.data[3] = wingsailAngle16 >> 8);
		(Cmsg.data[4] = 0);
		(Cmsg.data[5] = 0);
		(Cmsg.data[6] = m_windvaneSelfSteeringOn);
		(Cmsg.data[7] = 0);

		m_CANService->sendCANMessage(Cmsg);
	}

}