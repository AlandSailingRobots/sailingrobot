/****************************************************************************************
 *
 * File:
 * 		Xbee.h
 *
 * Purpose:
 *		Provides functions for interacting with a xbee radio device over usb.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#pragma once


#include <stdint.h>
#include <string>
#include <queue>
#include <mutex>


#define XBEE_BAUD_RATE 	57600
#define MAX_PACKET_SIZE 72

typedef void (*XbeeIncomingMsgFunc) (uint8_t* data, uint8_t size);

class Xbee {
public:
	Xbee(bool master);
	~Xbee();

	///----------------------------------------------------------------------------------
	/// Initialised the Xbee's handle and attempts to communicate with it to ensure it
	/// is present and working.
	///
	/// @param port 			The Xbee's port name, e.g. /dev/ttyUSB0.
	/// @param baudRate			The baud rate the Xbee is set to.
	///----------------------------------------------------------------------------------
	bool init(std::string port, uint16_t baudRate);

	//TODO - Jordan: Shared pointer maybe?

	///----------------------------------------------------------------------------------
	/// Sets the callback, the callback will take ownership of the data pointer passed to
	/// it.
	///----------------------------------------------------------------------------------
	void setIncomingCallback(XbeeIncomingMsgFunc func) { m_incomingCallback = func; }

	///----------------------------------------------------------------------------------
	/// Transmits a block of data
	///
	/// @param data				The data to transmit
	/// @param size				How many bytes to transmit.
	///----------------------------------------------------------------------------------
	void transmit(uint8_t* data, uint8_t size);

	///----------------------------------------------------------------------------------
	/// Receives any messages from the xbee and begins transmitting messages.
	///----------------------------------------------------------------------------------
	void processRadioMessages();

protected:
	// A Xbee packet has an overhead of 3 bytes
	struct XbeePacket {
		uint8_t m_packetID;
		uint8_t m_packetCount;
		uint8_t m_payloadSize;
		uint8_t* m_payload;
	};

	///----------------------------------------------------------------------------------
	/// Writes a packet to the xbee. This function is virtual so the mock unit test
	/// framework can override them.
	///----------------------------------------------------------------------------------
	virtual void writeData(XbeePacket packet);

	void writeData(uint8_t* data, uint8_t size);

	///----------------------------------------------------------------------------------
	/// Reads data from the Xbee serial line into a provided packet. This function also
	/// returns the number of bytes that were read.
	///----------------------------------------------------------------------------------
	virtual uint8_t readData(XbeePacket& packet);
	void readData(uint8_t* data, uint8_t size);

	///----------------------------------------------------------------------------------
	/// Returns true if there is data to read.
	///----------------------------------------------------------------------------------
	bool dataAvailable();

	///----------------------------------------------------------------------------------
	/// Attempts to read packets and stores them for processing. Returns true if any
	/// packets were received.
	///----------------------------------------------------------------------------------
	bool receivePackets();

	///----------------------------------------------------------------------------------
	/// Processes the packets, joins multi-packet messages and checks for malformed data.
	///----------------------------------------------------------------------------------
	void processPacketQueue();

	void processPacket(XbeePacket& packet);
	void processPacket(std::vector<XbeePacket>& packets);

	bool					m_master;
	int 					m_handle; // Xbee fd
	bool					m_initialised;
	int 					m_currPacketID;
	std::queue<XbeePacket>	m_receiveQueue;
	std::queue<XbeePacket>	m_transmitQueue;
	std::mutex				m_transmitQueueMutex;
	int						m_packetsReceived;
	int						m_badPackets;
	XbeeIncomingMsgFunc		m_incomingCallback;
};
