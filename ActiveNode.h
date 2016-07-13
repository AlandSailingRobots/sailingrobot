/****************************************************************************************
 *
 * File:
 * 		ActiveNode.h
 *
 * Purpose:
 *		A active node is a base(passive) node that has a thread.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "Node.h"
#include <thread>

typedef void (*ActiveNodeFunc) (void*); 


class ActiveNode : public Node {
public:
	ActiveNode(NodeID id, MessageBus& msgBus) : Node(id,msgBus)
	{ }

	virtual void processMessage(const Message* message) = 0;
protected:
	void runThread(ActiveNodeFunc func);
private:
	std::thread* m_Thread;
};