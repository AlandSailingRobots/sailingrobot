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

#include "MessageBus/Node.h"
#include "DataBase"

class ActuatorNode : public Node {
public:
	ActuatorNode(MessageBus& msgBus, DBHandler& dbhandler, NodeID id, int channel, int speed, int acceleration);

	///----------------------------------------------------------------------------------
 	/// Setups the actuator.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init();

	void updateConfigsFromDB();

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
