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
#include "logger/Logger.h"


ActuatorNode::ActuatorNode(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration)
	:Node(id, msgBus), m_Channel(channel), m_Speed(speed), m_Acceleration(acceleration)
{

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
		Logger::error("%s Failed to write actuator speed and aceleration!", __PRETTY_FUNCTION__);
		return false;
	}

}

void ActuatorNode::processMessage(const Message* message)
{
	if(message->messageType() == MessageType::ActuatorPosition)
	{
		ActuatorPositionMsg* msg = (ActuatorPositionMsg*)message;

		if(not MaestroController::writeCommand(MaestroCommands::SetPosition, m_Channel, msg->position()))
		{
			Logger::error("%s Actuator: %d Failed to write position command", __PRETTY_FUNCTION__, (int)nodeID());
		}
	}

}
