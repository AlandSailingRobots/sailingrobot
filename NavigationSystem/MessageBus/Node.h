/****************************************************************************************
 *
 * File:
 * 		Node.h
 *
 * Purpose:
 *		The base passive node.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

class MessageBus;

#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/NodeIDs.h"

enum NodeStatusFlag
{
    FLAG_INIT_FAIL = 0 << 0,
    RUNNING = 1 << 0, // binary 0001
    IDLE = 1 << 1, // binary 0010
    FROZEN = 1 << 2, // binary 0100
    WAITING = 1 << 3, // binary 1000
};

class Node {
   public:
    Node(NodeID id, MessageBus& msgBus) : m_MsgBus(msgBus), m_NodeID(id) {}

    virtual ~Node(){};

    ///----------------------------------------------------------------------------------
    /// This function should attempt to initialise or setup any resources it may need to
    /// function. If this was successful this function should return true.
    ///
    ///----------------------------------------------------------------------------------
    virtual bool init() = 0;

    ///----------------------------------------------------------------------------------
    /// Called by the MessageBus when it has a message the node might be interested in.
    /// A node should register for messages it wants to receive using
    /// MessageBus::registerNode(Node*, MessageType)
    ///
    ///----------------------------------------------------------------------------------
    virtual void processMessage(const Message* message) = 0;

    ///----------------------------------------------------------------------------------
    /// This function should get the last configuring values from the DataBase
    /// That enables the user to change this values by the web interface
    /// which changes the behaviour of the Node and thus the sailing robot behaviour.
    ///
    ///----------------------------------------------------------------------------------
    virtual void updateConfigsFromDB(){};

    ///----------------------------------------------------------------------------------
    /// Check the status of the node.
    /// Currently implemented for: HMC6343, 
    ///----------------------------------------------------------------------------------
    virtual bool updateStatus(NodeStatusFlag* nodeStatus) { return true; }

    NodeID nodeID() { return m_NodeID; }

    NodeStatusFlag nodeStatus() { return m_nodeStatus; }

   protected:
    MessageBus& m_MsgBus;
    NodeStatusFlag m_nodeStatus;

   private:
    NodeID m_NodeID;
};
