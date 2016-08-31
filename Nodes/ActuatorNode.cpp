/****************************************************************************************
 *
 * File:
 * 		ActuatorNode.cpp
 *
 * Purpose:
 *		Controls an actuator on the vessel.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "ActuatorNode.h"
#include "Messages/ActuatorPositionMsg.h"
#include "SystemServices/MaestroController.h"
#include "SystemServices/Logger.h"


// HARD LIMITS

#define SAIL_MAX_US		0
#define SAIL_MIN_US		0

#define RUDDER_MAX_US	0
#define RUDDER_MIN_US	0



ActuatorNode::ActuatorNode(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration)
	:Node(id, msgBus), m_Channel(channel), m_Speed(speed), m_Acceleration(acceleration)
{
  msgBus.registerNode(*this,MessageType::ActuatorPosition);
}

bool ActuatorNode::init()
{
	if( MaestroController::writeCommand(MaestroCommands::SetSpeed, m_Channel, m_Speed) &&
		MaestroController::writeCommand(MaestroCommands::SetAcceleration, m_Channel, m_Acceleration) )
	{
		return true;
	}
	else
	{
		Logger::error("%s Failed to write actuator speed and acceleration!", __PRETTY_FUNCTION__);
		return false;
	}

}

void ActuatorNode::processMessage(const Message* message)
{
	if(message->messageType() == MessageType::ActuatorPosition)
	{
		ActuatorPositionMsg* msg = (ActuatorPositionMsg*)message;

		if (nodeID() == NodeID::RudderActuator)
		{
			int value = msg->rudderPosition();
			if(value > RUDDER_MIN_US && value < RUDDER_MAX_US)
			{
				sendCommand(value);
			}
		}
		else if (nodeID() == NodeID::SailActuator)
		{
			int value = msg->sailPosition();
			if(value > RUDDER_MIN_US && value < RUDDER_MAX_US)
			{
				sendCommand(value);
			}
		}
		else
		{
			Logger::warning("%s Actuator: %d Unknown/Unregistered actuator message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
			return;
		}
	}
}

void ActuatorNode::sendCommand(int value)
{
	if(not MaestroController::writeCommand(MaestroCommands::SetPosition, m_Channel, setPosition))
	{
		Logger::error("%s Actuator: %d Failed to write position command", __PRETTY_FUNCTION__, (int)nodeID());
	}
}
