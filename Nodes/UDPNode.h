/****************************************************************************************
 *
 * File:
 * 		UDPNode.h
 *
 * Purpose:
 *		The purpose of the UDP Node is to send subscribed messages over a UDP link to an
 *		external applications.
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include "Node.h"
#include "../XbeeRemote/UDPRelay.h"


class UDPNode : public Node {
public:
	UDPNode(MessageBus& msgBus, std::string address, int port);
	virtual ~UDPNode();

	///----------------------------------------------------------------------------------
	/// This function setups the UDP socket.
	///----------------------------------------------------------------------------------
	virtual bool init();

	///----------------------------------------------------------------------------------
	/// Sends a received message over UDP
	///----------------------------------------------------------------------------------
	virtual void processMessage(const Message* message);
private:
	UDPRelay 	m_udpRelay;
};
