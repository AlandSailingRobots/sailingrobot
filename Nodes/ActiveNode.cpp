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

#include "ActiveNode.h"


void ActiveNode::runThread(ActiveNodeFunc func)
{
	m_ThreadPtr = new std::thread(func, this);
}