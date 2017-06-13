/****************************************************************************************
 *
 * File:
 * 		ActiveNode.cpp
 *
 * Purpose:
 *		A active node is a base(passive) node that has a thread.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "MessageBus/ActiveNode.h"

#include <iostream>
void ActiveNode::runThread(void(*func)(ActiveNode*))
{
	m_Thread = new std::thread(func, this);
}