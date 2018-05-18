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

#include <thread>
#include "Node.h"

class ActiveNode : public Node {
   public:
    ActiveNode(NodeID id, MessageBus& msgBus) : Node(id, msgBus) {}

    ///----------------------------------------------------------------------------------
    /// This function should be used to start the active nodes thread.
    ///
    ///----------------------------------------------------------------------------------
    virtual void start() = 0;

   protected:
    void runThread(void (*func)(ActiveNode*));
    void stopThread(ActiveNode* node);

   private:
    std::thread* m_Thread;
};
