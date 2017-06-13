/****************************************************************************************
 *
 * File:
 * 		MessageLoggerNode.h
 *
 * Purpose:
 *		The message logger node, logs all message that come through the system.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Node.h"


class MessageLoggerNode : public Node {
public:
	MessageLoggerNode(MessageBus& msgBus);

	bool init();

	void processMessage(const Message* message);
private:
};