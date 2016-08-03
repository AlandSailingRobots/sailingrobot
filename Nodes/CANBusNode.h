/****************************************************************************************
 *
 * File:
 * 		ActuatorNode.h
 *
 * Purpose:
 *		Controls an actuator on the vessel.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Node.h"

class CANBus : public Node {
public:
	CANBusNode(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration);

	///----------------------------------------------------------------------------------
 	/// Setups the actuator.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init();

	///----------------------------------------------------------------------------------
 	/// Processes ActuatorPositionMsgs
 	///
 	///----------------------------------------------------------------------------------
	virtual void processMessage(const Message* message);

private:
	uint16_t m_rudderPosition;
	uint16_t m_sailPosition;
	uint8_t m_windVaneCommand;
	bool m_actuatorPositionsReady;
	bool m_windVaneCommandReady;
};
