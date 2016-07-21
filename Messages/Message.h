/****************************************************************************************
 *
 * File:
 * 		Message.h
 *
 * Purpose:
 *		Provides the base message class
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageTypes.h"
#include "NodeIDs.h"

class Message {
public:
	///----------------------------------------------------------------------------------
 	/// Default constructor
 	///
 	/// @param msgType 			The type of message this is.
 	/// @param msgSource		Which node posted this message.
 	/// @param msgDest			Which node the message should go to. Leave this as
 	///							NodeIDS::NONE for no specific destination.
 	///----------------------------------------------------------------------------------
	Message(MessageType msgType, NodeID msgSource, NodeID msgDest)
		:m_MessageType(msgType), m_SourceID(msgSource), m_DestinationID(msgDest)
	{ }

	///----------------------------------------------------------------------------------
 	/// A message with no specific destination.
 	///
 	/// @param msgType 			The type of message this is.
 	/// @param msgSource		Which node posted this message.
 	///----------------------------------------------------------------------------------
	Message(MessageType msgType, NodeID msgSource)
		:m_MessageType(msgType), m_SourceID(msgSource), m_DestinationID(NodeID::None)
	{ }

	///----------------------------------------------------------------------------------
 	/// A message with no specific destination or source.
 	///
 	/// @param msgType 			The type of message this is.
 	///----------------------------------------------------------------------------------
	Message(MessageType msgType)
		:m_MessageType(msgType), m_SourceID(NodeID::None), m_DestinationID(NodeID::None)
	{ }

	virtual ~Message() { }

	///----------------------------------------------------------------------------------
 	/// Returns the type of message this is.
 	///
 	///----------------------------------------------------------------------------------
	MessageType messageType() const { return m_MessageType; }

	///----------------------------------------------------------------------------------
 	/// Returns the node which generated this message.
 	///
 	///----------------------------------------------------------------------------------
	NodeID sourceID() const { return m_SourceID; }

	///----------------------------------------------------------------------------------
 	/// Returns the message's destination.
 	///
 	///----------------------------------------------------------------------------------
	NodeID destinationID() const { return m_DestinationID; }

private:
	MessageType m_MessageType;		// The message type
	NodeID m_SourceID;				// Which node generated the message
	NodeID m_DestinationID;			// The desintation of the message
};