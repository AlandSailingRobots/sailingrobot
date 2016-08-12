/****************************************************************************************
 *
 * File:
 * 		XbeePacketNetwork.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include "DataLink.h"
#include <stdint.h>
#include <string>
#include <queue>
#ifndef _WIN32
#include <mutex>
#endif

#define XBEE_BAUD_RATE 	57600
#define XBEE_PACKET_SIZE 70 // Its actually 72, but 2 bytes will be used for framing

typedef void (*XbeeIncomingMsgFunc) (uint8_t* data, uint8_t size);


class XbeePacketNetwork {
public:
	XbeePacketNetwork(DataLink& dataLink, bool master);
	virtual ~XbeePacketNetwork();

	///----------------------------------------------------------------------------------
	/// Receives any messages from the xbee and begins transmitting messages.
	///----------------------------------------------------------------------------------
	void processRadioMessages();

	///----------------------------------------------------------------------------------
	/// Transmits a block of data, a copy is made of the data.
	///
	/// @param data				The data to transmit
	/// @param size				How many bytes to transmit.
	///----------------------------------------------------------------------------------
	void transmit(uint8_t* data, uint8_t size);

	///----------------------------------------------------------------------------------
	/// Sets the callback, the callback will take ownership of the data pointer passed to
	/// it.
	///----------------------------------------------------------------------------------
	void setIncomingCallback(XbeeIncomingMsgFunc func) { m_incomingCallback = func; }

	///----------------------------------------------------------------------------------
	/// Returns the fletcher's checksum of a block of bytes.
	///----------------------------------------------------------------------------------
	uint16_t fletcherChecksum(uint8_t* data, uint16_t size);

protected:
	// A Xbee packet has an overhead of 3 bytes
	struct XbeePacket {
		uint8_t m_packetID;
		uint8_t m_packetCount;
		uint8_t m_payloadSize;
		uint8_t* m_payload;
		uint16_t m_checksum;
		bool m_ownsMem;
	};

	///----------------------------------------------------------------------------------
	/// Attempts to receive a packet and if it does so successful it is stored in the
	/// receive queue.
	///----------------------------------------------------------------------------------
	bool receivePacket();

	///----------------------------------------------------------------------------------
	/// Actually transmit a packet across the data link
	///----------------------------------------------------------------------------------
	void transmitPackets(uint8_t packetsToSend);

	///----------------------------------------------------------------------------------
	/// Listen for packets over the data link
	///----------------------------------------------------------------------------------
	void processReceivedPackets();
	void processPacket(XbeePacket& packet);
	void processPacket(std::vector<XbeePacket>& packets);

private:
	DataLink& 				m_dataLink;
	bool					m_master;
	int 					m_currPacketID;
	std::queue<XbeePacket>	m_receiveQueue;
	std::queue<XbeePacket>	m_transmitQueue;
	#ifndef _WIN32
	std::mutex				m_transmitQueueMutex;
	#endif
	int						m_packetsReceived;
	int						m_badPackets;
	XbeeIncomingMsgFunc		m_incomingCallback;
};
