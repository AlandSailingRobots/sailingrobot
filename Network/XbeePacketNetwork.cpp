/****************************************************************************************
 *
 * File:
 * 		XbeePacketNetwork.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "XbeePacketNetwork.h"
#include "utility/SysClock.h"
#include "SLIP.h"
#include "SystemServices/Logger.h"
#include <cstring>

// For std::this_thread
#include <chrono>
#include <thread>


#define XBEE_PACKET_OVERHEAD 		6
#define XBEE_PACKET_MAX_OVERHEAD 	6*2 // START character(1), Packet ID(1), packet count(1), payload size(1), checksum(2), slip could double the size

#define XBEE_TRANSMIT_TIME 			50


XbeePacketNetwork::XbeePacketNetwork(DataLink& dataLink, bool master)
	:m_dataLink(dataLink), m_master(master), m_currPacketID(0)
{

}

XbeePacketNetwork::~XbeePacketNetwork()
{
	// TODO Auto-generated destructor stub
}

void XbeePacketNetwork::processRadioMessages()
{
	static unsigned long int lastReceived = 0;
	const int TRANSMIT_WAIT = 1; // 1 seconds, should probably be less, like 500ms
	const uint8_t PACKETS_TO_TRANSMIT = 5;
	const uint8_t MAX_PACKETS_TO_RECEIVE = 10;

	// Attempt to receive and process packets
	bool received = false;
	for(uint8_t i = 0; i < MAX_PACKETS_TO_RECEIVE; i++)
	{
		if(receivePacket()) { received = true; }
	}

	if(received)
	{
		lastReceived = SysClock::unixTime();
	}

	processReceivedPackets();

	// Transmit any stored packets
	if(m_transmitQueue.size() > 0)
	{
		// Locks until the function returns and the current scope is left
		std::lock_guard<std::mutex> lock(m_transmitQueueMutex);

		if(m_master)
		{
			transmitPackets(0);
		}
		// When we are not in master mode we should only transmit a few packets then break incase the master
		// wants to interrupt.
		else if((SysClock::unixTime() - lastReceived) > TRANSMIT_WAIT)
		{
			transmitPackets(PACKETS_TO_TRANSMIT);
		}
	}
}

void XbeePacketNetwork::transmit(uint8_t* data, uint8_t size)
{
	const uint8_t MAX_DATA_SIZE = XBEE_PACKET_SIZE - XBEE_PACKET_MAX_OVERHEAD;
	uint8_t packetCount = SLIP::packetCount(data, size, MAX_DATA_SIZE);
	uint16_t bytesToProcess = 0;
	uint8_t* currPtr = data;

	// Locks until the function returns and the current scope is left
	std::lock_guard<std::mutex> lock(m_transmitQueueMutex);

	for(uint8_t i = 0; i < packetCount; i++)
	{
		XbeePacket packet;
		packet.m_packetID = m_currPacketID;
		packet.m_packetCount = packetCount;
		packet.m_payloadSize = SLIP::splitSize(currPtr, size, MAX_DATA_SIZE, bytesToProcess);
		packet.m_payload = currPtr;
		packet.m_checksum = fletcherChecksum(packet.m_payload, packet.m_payloadSize);

		// The first packet actually owns the memory pointer.
		packet.m_ownsMem = (i == 0);

		m_transmitQueue.push(packet);
		currPtr = currPtr + packet.m_payloadSize;
	}

	m_currPacketID++;
}

uint16_t XbeePacketNetwork::fletcherChecksum(uint8_t* data, uint16_t size)
{
	// Taken for Wikipedia - https://en.wikipedia.org/wiki/Fletcher's_checksum
	uint16_t sum1 = 0xff, sum2 = 0xff;
	size_t tlen;

	while (size) {
			tlen = size >= 20 ? 20 : size;
			size -= tlen;
			do {
					sum2 += sum1 += *data++;
			} while (--tlen);
			sum1 = (sum1 & 0xff) + (sum1 >> 8);
			sum2 = (sum2 & 0xff) + (sum2 >> 8);
	}
	/* Second reduction step to reduce sums to 8 bits */
	sum1 = (sum1 & 0xff) + (sum1 >> 8);
	sum2 = (sum2 & 0xff) + (sum2 >> 8);
	return sum2 << 8 | sum1;
}

bool XbeePacketNetwork::receivePacket()
{
	const uint8_t PAYLOAD_START = 3;

	NetworkFrame frame;
	bool received = false;

	if(m_dataLink.receive(frame))
	{
		if(frame.m_size > XBEE_PACKET_OVERHEAD)
		{
			m_packetsReceived++;

			XbeePacket packet;
			packet.m_packetID = frame.m_data[0];
			packet.m_packetCount = frame.m_data[1];
			packet.m_payloadSize = frame.m_data[2];

			uint16_t payloadLength = frame.m_size - XBEE_PACKET_OVERHEAD;
			packet.m_payload = new uint8_t[payloadLength];
			memcpy(packet.m_payload, frame.m_data + PAYLOAD_START, payloadLength);

			// LSB is sent first, followed by the MSB
			packet.m_checksum = (frame.m_data[frame.m_size - 1] << 8) | frame.m_data[frame.m_size - 2];

			// Check the checksum and cleanup if the checksum was bad
			if(packet.m_checksum != fletcherChecksum(packet.m_payload, packet.m_payloadSize))
			{
				delete[] packet.m_payload;
				m_badPackets++;
			}
			else
			{
				received = true;
				m_receiveQueue.push(packet);
			}
		}
	}

	return received;
}

void XbeePacketNetwork::transmitPackets(uint8_t packetsToSend)
{
	uint8_t packetCount = (packetsToSend == 0 ? m_transmitQueue.size() : packetsToSend);
	uint8_t packetsLeft = 0;
	uint8_t* ptrToClean = NULL;

	for(uint8_t i = 0; i < packetCount; i++)
	{
		XbeePacket& packet = m_transmitQueue.front();
		uint8_t frameSize = packet.m_payloadSize + XBEE_PACKET_OVERHEAD;

		uint8_t* frame = new uint8_t[packet.m_payloadSize + XBEE_PACKET_OVERHEAD];
		frame[0] = packet.m_packetID;
		frame[1] = packet.m_packetCount;
		frame[2] = packet.m_payloadSize;
		memcpy(frame + 3, packet.m_payload, packet.m_payloadSize);
		frame[frameSize - 2] = (uint8_t)(packet.m_checksum & 0xFFu);
		frame[frameSize - 1] = (uint8_t)( (packet.m_checksum >> 8) & 0xFFu );

		// Pointer ownership is passed onto netFrame
		NetworkFrame netFrame(frame, frameSize);
		m_dataLink.transmit(netFrame);

		std::this_thread::sleep_for(std::chrono::milliseconds(XBEE_TRANSMIT_TIME));

		if(packetsLeft != 0)
		{
			packetsLeft--;
		}

		// If this is a one packet message, we can just clean it up
		if(packet.m_packetCount == 1)
		{
			delete[] packet.m_payload;
			packet.m_payload = NULL;
		}
		// If the message is made up of multiple packets, only one packet will have the actual
		// valid memory pointer that can be deleted
		else if(packet.m_packetCount > 1 && packet.m_ownsMem)
		{
			packetsLeft = packet.m_packetCount;
			ptrToClean = packet.m_payload;
		}
		else if(packetsLeft == 0)
		{
			delete[] ptrToClean;
			ptrToClean = NULL;
		}

		m_transmitQueue.pop();
	}
}

void XbeePacketNetwork::processReceivedPackets()
{
	// Nothing to do if there are no packets to process
	if(m_receiveQueue.size() == 0)
	{
		return;
	}

	uint8_t lastPacketID = m_receiveQueue.front().m_packetID;
	unsigned int queueSize = m_receiveQueue.size();

	std::vector<XbeePacket> multiPackets;

	for(unsigned int i = 0; i < queueSize; i++)
	{
		XbeePacket packet = m_receiveQueue.front();

		// Complete multi-packet, process it
		if(multiPackets.size() > 0 && packet.m_packetID != lastPacketID)
		{
			processPacket(multiPackets);
			multiPackets.clear();
		}

		// A multi packet, wait for the others
		if(packet.m_packetCount > 1)
		{
			multiPackets.push_back(packet);
		}
		// Just a single packet
		else
		{
			processPacket(packet);
		}
		m_receiveQueue.pop();
	}

	if(multiPackets.size() > 0)
	{
		if(multiPackets.size() == multiPackets[0].m_packetCount)
		{
			processPacket(multiPackets);
		}
		// Could be more packets following
		else
		{
			for(auto p : multiPackets)
			{
				m_receiveQueue.push(p);
			}
		}
	}
}

void XbeePacketNetwork::processPacket(XbeePacket& packet)
{
	if(m_incomingCallback != NULL)
	{
		//Logger::info("Single inbound packet Size: %d", packet.m_payloadSize);

		m_incomingCallback(packet.m_payload, packet.m_payloadSize);
	}
	else
	{
		Logger::info("No callback set!");
		// Clean up if there is no one to hand the data to.
		delete[] packet.m_payload;
	}
}

void XbeePacketNetwork::processPacket(std::vector<XbeePacket>& packets)
{
	// No one to hand packets to, so cleanup
	if(m_incomingCallback == NULL || packets.size() != packets[0].m_packetCount)
	{
		Logger::info("Missing packets or no callback set!");
	}
	else
	{
		// Work out the full message size
		uint8_t payloadSize = 0;
		for(auto p : packets)
		{
			payloadSize += p.m_payloadSize;
		}

		// Copy the data into one block
		uint8_t* finalPayload = new uint8_t[payloadSize];
		uint8_t* copyPtr = finalPayload;
		for(auto p : packets)
		{
			memcpy(copyPtr, p.m_payload, p.m_payloadSize);
			copyPtr += p.m_payloadSize;
		}

		m_incomingCallback(finalPayload, payloadSize);
	}

	// Clean up the packets
	for(auto p : packets)
	{
		delete[] p.m_payload;
	}

	packets.clear();
}
