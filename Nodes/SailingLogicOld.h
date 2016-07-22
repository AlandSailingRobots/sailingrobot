/****************************************************************************************
 *
 * File:
 * 		SailigLogicOld.h
 *
 * Purpose:
 *		Calculate the direction to the waypoint
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Node.h"


class MessageLoggerNode : public Node {
public:
	MessageLoggerNode(MessageBus& msgBus);

	bool init();

	void processMessage(const Message* message);
private:
};