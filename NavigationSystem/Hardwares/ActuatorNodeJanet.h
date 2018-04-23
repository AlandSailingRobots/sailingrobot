/****************************************************************************************
 *
 * File:
 * 		ActuatorNodeJanet.h
 *
 * Purpose:
 *		Controls an actuator on the vessel.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Node.h"
#include "DataBase/DBHandler.h"

class ActuatorNodeJanet : public Node {
public:
	ActuatorNodeJanet(MessageBus& msgBus, DBHandler& dbhandler, NodeID id, int channel, int speed, int acceleration);

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
	int m_Channel;
	int m_Speed;
	int m_Acceleration;
	DBHandler& m_db;
};
