/****************************************************************************************
 *
 * File:
 * 		MessageBus.h
 *
 * Purpose:
 *		The message bus manages message distribution to nodes allowing nodes to
 *		communicate with one another.
 *
 * Developer Notes:
 *		Nodes can only be added before the run function is called currently. This is to
 *		reduce the number of thread locks in place and because once the system has 
 *		started its very rare that a node should be registered afterwards on the fly.
 *
 *
 ***************************************************************************************/

#pragma once

#include "Node.h"
#include "Messages/Message.h"
#include <vector>
#include <queue>
#include <mutex>
#include <iostream>
#include <fstream>

class Node;


class MessageBus {
public:
	///----------------------------------------------------------------------------------
 	/// Default constructor
 	///
 	///----------------------------------------------------------------------------------
	MessageBus();

	///----------------------------------------------------------------------------------
	/// We really don't need to do memory cleanup here as this class will only be
	/// cleaned up when the program ends
	///----------------------------------------------------------------------------------
	~MessageBus();

	///----------------------------------------------------------------------------------
 	/// Registers a node onto the message bus allowing it receive direct messages. The 
 	/// message bus does not own the node.
 	///
 	/// @param node 			Pointer to the node that should be registered.
 	///----------------------------------------------------------------------------------
	bool registerNode(Node* node);

	///----------------------------------------------------------------------------------
 	/// Registers a node onto the message bus allowing it receive direct messages and
 	/// also subscribes it for a particular message type.
 	///
 	/// @param node 			Pointer to the node that should be registered.
 	/// @param msgType 			The type of message to register for
 	///----------------------------------------------------------------------------------
	bool registerNode(Node* node, MessageType msgType);

	///----------------------------------------------------------------------------------
 	/// Enqueues a message onto the message queue for distribution through the message 
 	/// bus.
 	///
 	/// @param msg 				Pointer to the message that should be enqeued, this
 	///							passes ownership to the MessageBus.
 	///----------------------------------------------------------------------------------
	void sendMessage(Message* msg);

	///----------------------------------------------------------------------------------
 	/// Begins running the message bus and distributing messages to nodes that have been
 	/// registered. This function won't ever return.
 	///----------------------------------------------------------------------------------
	void run();
private:
	///----------------------------------------------------------------------------------
 	/// Stores information about a registered node and the message types it is interested
 	/// in.
 	///----------------------------------------------------------------------------------
	struct RegisteredNode {
		RegisteredNode(Node* node) : nodePtr(node) { }

		Node* nodePtr;

		///------------------------------------------------------------------------------
 		/// Returns true if a registered node is interested in a message type.
 		///
 		/// @param type 		The message type which is checked for.
 		///------------------------------------------------------------------------------
		bool isInterested(MessageType type)
		{
			for(auto msgType : interestedList) { 
				if(type == msgType) { return true; } 
			}
			return false;
		}

		///------------------------------------------------------------------------------
 		/// Subscribes the registered node for a particular type of message
 		///
 		/// @param type 		The message type to subscribe this registered node to.
 		///------------------------------------------------------------------------------
		void subscribe(MessageType type) 
		{
			// Maintain only one copy of each interested type.
			if(not isInterested(type)) { interestedList.push_back(type); }
		}


	private:
		std::vector<MessageType> interestedList;
	};

	///----------------------------------------------------------------------------------
 	/// Looks for existing registered node for a given node pointer and returns a pointer
 	/// to it. If the node has not yet been registered, it is then registered and a 
 	/// pointer to the new RegisteredNode is returned.
 	///----------------------------------------------------------------------------------
	RegisteredNode* getRegisteredNode(Node* node);

	///----------------------------------------------------------------------------------
 	/// Goes through the back message queue and distributes messages, calling 
 	/// Node::processMessage(Message*) on nodes that are interested in any given message.
 	///----------------------------------------------------------------------------------
	void processMessages();

	///----------------------------------------------------------------------------------
	/// Creates a log file for the messages.
	///----------------------------------------------------------------------------------
	void startMessageLog();

	///----------------------------------------------------------------------------------
	/// Logs that a message was received.
	///----------------------------------------------------------------------------------
	void logMessageReceived(Message* msg);

	///----------------------------------------------------------------------------------
	/// Logs a message that is being processed.
	///----------------------------------------------------------------------------------
	void logMessage(Message* msg);

	///----------------------------------------------------------------------------------
	/// Logs that a node consumed the current message being processed.
	///----------------------------------------------------------------------------------
	void logMessageConsumer(NodeID id);

	///----------------------------------------------------------------------------------
	/// Logs that the current message being processed has finished being processed.
	///----------------------------------------------------------------------------------
	//void logMessageProcessed(Message* msg);

	///----------------------------------------------------------------------------------
	/// Returns a message time stamp in the format HH:MM:SS:MS, a char buffer of 14
	/// characters needs to be provided.
	///----------------------------------------------------------------------------------
	void messageTimeStamp(unsigned long unixTime, char* buffer);

	bool 							m_Running;
	std::vector<RegisteredNode*> 	m_RegisteredNodes;
	std::queue<Message*>* 			m_FrontMessages; 	// The forward facing message queue 
													 	// which messages are append to.
	std::queue<Message*>*			m_BackMessages; 	// The backend message queue which 
														// contains messages to distribute.
	std::mutex						m_FrontQueueMutex;	// Guards the front message queue.
	
#ifdef LOG_MESSAGES
	std::ofstream* 					m_LogFile;
#endif
};
