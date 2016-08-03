/****************************************************************************************
 *
 * File:
 * 		CANBusNode.cpp
 *
 * Purpose:
 *		Processes messages on their way to or coming back from the Arduino which 
		are communicated over the CAN bus.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "CANBusNode.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/ActuatorFeedbackMsg.h"
#include "Messages/CANMessageIDs.h"
#include "Messages/WindDataMsg.h"
#include "SystemServices/Logger.h"


CANBusNode::CANBusNode(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration)
	:Node(id, msgBus)
{
	msgBus.registerNode(this,MessageType::CANBusFeedback);
	msgBus.registerNode(this,MessageType::ActuatorCommand);

}

bool CANBusNode::init()
{
	m_actuatorPositionsReady = false;
	m_windVanceCommandReady = false;
}

void CANBusNode::sendToArduino(uint32_t dataID, uint32_t CANID, uint8_t* CANData)
{
	//TODO
}

void CANBusNode::loadOutgoingMessage(const Message* message)
{
	if(message->messageType() == MessageType::ActuatorPosition)
	{
		ActuatorPositionMsg* msg = (ActuatorPositionMsg*)message;

		if (nodeID() == NodeID::CANBus)
		{
			//Get relevant command
			m_rudderPosition = msg->rudderPosition();
			m_sailPosition = msg->sailPosition();
		
		}
		
		else
		{
			Logger::warning("%s CANBus: %d Unknown/Unregistered CANBus message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
		}
		
		m_actuatorPositionsReady = true;
		
	}
	/*
	Not implemented but will probably be like this...

	elseif(message->messageType() == MessageType::WindVaneCommand)
	{
		WindVaneCommandMsg* msg = (WindVaneCommandMsg*)message;

		if (nodeID() == NodeID::CANBus)
		{
			//Get relevant command
			m_windVaneCommand = msg->windVaneCommand();
		
		}
		
		else
		{
			Logger::warning("%s Actuator: %d Unknown/Unregistered actuator message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
		}

		m_windVaneCommandReady = true;
		
	}
	*/
}

void CANBusNode::passToCANBus()
{
	if (m_actuatorPositionsReady)
	{
		uint8_t  CANData[8]
		uint32_t CANID = CANID::ActuatorCommandsOnly;

		CANData[0]= m_rudderPosition >> 8;
		CANData[1]= m_rudderPosition & 0x00ff;
		CANData[2]= m_sailPosition >> 8;
		CANData[3]= m_sailPosition & 0x00ff;

		if (m_windVaneCommandReady)
		{
			CANData[4]= m_windVaneCommand;
			CANID = CANID::ActuatorandWindVaneCommands;
		}

		sendToArduino(DataID::CANBus, CANID, CANData); //TODO
	
		m_actuatorPositionsReady = false;
		m_windVanceCommandReady = false;

	}
		
	else
	{
		Logger::warning("%s CANbus: %d Attempted to pass CAN commands with no actuator commands loaded.", __PRETTY_FUNCTION__, (int)nodeID());
	}
}


void CANBusNode::processIncomingCAN(uint32_t CANID, uint8_t* CANData)
{
	if(CANID == CANID::ActuatorFeedbackOnly)
	{
		uint16_t = rudderPosition = (CANData[0] << 8) | CANData[1];
		uint16_t = sailPosition = (CANData[2] << 8) | CANData[3];		

		ActuatorPositionMsg *actuatorMsg = new ActuatorPositionMsg(rudderPosition, sailPosition, true);
		m_MsgBus.sendMessage(actuatorMsg);
	}
	
	else if (CANID == CANID::ActuatorAndWindFeedback)
	{
		uint16_t = rudderPosition = (CANData[0] << 8) | CANData[1];
		uint16_t = sailPosition = (CANData[2] << 8) | CANData[3];
		uint8_t = windDir = //TODO;
		uint8_t = windSpeed = //TODO;
		uint16_t = windTemp = //TODO;

		ActuatorPositionMsg *actuatorMsg = new ActuatorPositionMsg(rudderCommand_norm, sailCommand_norm, true);
		m_MsgBus.sendMessage(actuatorMsg);

		WindDataMsg *windDataMsg = new WindDataMsg(windDir, windSpeed, windTemp);
		m_MsgBus.sendMessage(windDataMsg);
	}
}

