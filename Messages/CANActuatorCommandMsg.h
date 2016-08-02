/****************************************************************************************
 *
 * File:
 * 		ActuatorPositionMsg.h
 *
 * Purpose:
 *		A message contain an actuator's position.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"
#include "ActuatorPositionMsg.h"


class CANActuatorCommandMsg : public Message {
public:
	CANActuatorCommandMsg(NodeID destinationID, NodeID sourceID, ActuatorPositionMsg positions)
		:Message(MessageType::CANActuatorCommand, sourceID, destinationID), m_CANMsgBuf((positions.rudderPosition << 16) & positions.sailPosition())
	{ }

	CANActuatorCommandMsg(int rudderPosition, int sailPosition)
		:Message(MessageType::CANActuatorCommand, NodeID::None, NodeID::None), m_CANMsgBuf((positions.rudderPosition << 16) & positions.sailPosition())
	{ }

	virtual ~CANActuatorCommandMsg() { }

	int CANMsgBuf() { return m_CANMsgBuf; }
private:
	int m_CANMsgBuf
};
