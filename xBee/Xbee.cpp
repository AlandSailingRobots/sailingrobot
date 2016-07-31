/****************************************************************************************
 *
 * File:
 * 		Xbee.cpp
 *
 * Purpose:
 *		Provides functions for interacting with a xbee radio device over usb.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "Xbee.h"
#include "SystemServices/Logger.h"
#include <wiringSerial.h>
#include <thread>
#include <cstring>


#define AT_COMMAND_MODE_WAIT	3000
#define XBEE_TRANSMIT_TIME		50

#define PACKET_OVERHEAD			3




// REDO< FIX!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define ENTER_CMD_MODE() 	writeData((uint8_t*)"+++", 3); std::this_thread::sleep_for(std::chrono::milliseconds(AT_COMMAND_MODE_WAIT))
#define EXIT_CMD_MODE()		writeData((uint8_t*)"ATCN\r", 5)


Xbee::Xbee()
	:m_handle(-1), m_initialised(false), m_currPacketID(0)
{

}

Xbee::~Xbee()
{

}


bool Xbee::init(std::string port, uint16_t baudRate)
{
	std::string deviceName = "/dev/ttyUSB0";
	m_initialised = false;

	m_handle = serialOpen(port.c_str(), baudRate);

	if(m_handle < 0)
	{
		Logger::error("Failed to acquire a device handle for the Xbee");
		return m_initialised;
	}

	char buff[2];

	// See if the device will respond to a acknowledge
	ENTER_CMD_MODE();
	writeData((uint8_t*)"AT\r", 2);
	readData((uint8_t*)buff, sizeof(buff));
	EXIT_CMD_MODE();

	if(buff[0] == 'O' && buff[1] == 'K')
	{
		m_initialised = true;
		return m_initialised;
	}
	else
	{
		Logger::error("Failed to receive OK from xbee!");
		return m_initialised;
	}
}

void Xbee::transmit(uint8_t* data, uint8_t size)
{
	const uint8_t maxDataSize = MAX_PACKET_SIZE - PACKET_OVERHEAD;
	// Split the packet
	if(size > maxDataSize)
	{
		// Round up
		uint8_t packetCount = 1 + ((size - 1) / maxDataSize);
		uint8_t* dataPtr = data;

		for(uint8_t i = 0; i < packetCount - 1; i++)
		{
			XbeePacket packet;
			packet.m_packetID = m_currPacketID;
			packet.m_packetCount = packetCount;
			packet.m_payloadSize = maxDataSize;
			packet.m_payload = new uint8_t[maxDataSize];
			memcpy(packet.m_payload, dataPtr, maxDataSize);
			dataPtr = dataPtr + maxDataSize;

			writeData(packet);
		}

		XbeePacket packet;
		packet.m_packetID = m_currPacketID;
		packet.m_packetCount = packetCount;
		packet.m_payloadSize = size - ((packetCount - 1) * maxDataSize);
		packet.m_payload = new uint8_t[packet.m_payloadSize];
		memcpy(packet.m_payload, dataPtr, packet.m_payloadSize);

		writeData(packet);

		m_currPacketID++;
	}
	else
	{
		XbeePacket packet;
		packet.m_packetID = m_currPacketID++; // CPP Note: incremented after assignment
		packet.m_packetCount = 1;
		packet.m_payloadSize = size;
		packet.m_payload = data;

		writeData(packet);
	}
}

void Xbee::writeData(XbeePacket packet)
{
	serialPutchar(m_handle, (char)packet.m_packetID);
	serialPutchar(m_handle, (char)packet.m_packetCount);
	serialPutchar(m_handle, (char)packet.m_payloadSize);

	writeData(packet.m_payload, packet.m_payloadSize);

	// TODO - Jordan: Test this value and reliability.
	std::this_thread::sleep_for(std::chrono::milliseconds(XBEE_TRANSMIT_TIME));
}

void Xbee::writeData(uint8_t* data, uint8_t size)
{
	for(uint8_t index = 0; index < size; index++)
	{
		serialPutchar(m_handle, (char)data[index]);
	}
}

uint8_t Xbee::readData(XbeePacket& packet)
{
	int bytesAvailable = serialDataAvail(m_handle);
	uint8_t bytesRead = 0;

	// The packet must have more data than the packet overhead
	if(bytesAvailable > PACKET_OVERHEAD)
	{
		bytesAvailable = bytesAvailable - 3;
		packet.m_packetID = serialGetchar(m_handle);
		packet.m_packetCount = serialGetchar(m_handle);
		packet.m_payloadSize = serialGetchar(m_handle);

		// Check all the payload made it
		if(bytesAvailable >= packet.m_payloadSize)
		{
			packet.m_payload = new uint8_t[packet.m_payloadSize];

			Xbee::readData(packet.m_payload, packet.m_payloadSize);
			bytesRead = packet.m_payloadSize + PACKET_OVERHEAD;;
		}
		else
		{
			Logger::error("%s Malformed packet - Less bytes available than the packet payload", __PRETTY_FUNCTION__);
		}

	}
	else
	{
		Logger::warning("%s No Data - Failed to receive data from the Xbee", __PRETTY_FUNCTION__);
	}

	return bytesRead;
}

void Xbee::readData(uint8_t* data, uint8_t size)
{
	for(uint8_t bytesRead = 0; bytesRead < size; bytesRead++)
	{
		data[bytesRead] = serialGetchar(m_handle);
	}
}

bool Xbee::dataAvailable()
{
	return serialDataAvail(m_handle);
}

void Xbee::receivePackets()
{
	// Check for packets to be available and if there isn't anything to do with
	// the packets, don't bother receiving them
	if(not dataAvailable() || m_incomingCallback == NULL)
	{
		return;
	}

	bool readPackets = true;

	// TODO - Jordan: Add a time check here too just in case
	while(readPackets)
	{
		XbeePacket packet;

		if(readData(packet))
		{
			m_receiveQueue.push(packet);
			m_packetsReceived++;
		}
		else
		{
			readPackets = false;
		}
	}
}

void Xbee::processPacketQueue()
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

void Xbee::processPacket(XbeePacket& packet)
{
	if(m_incomingCallback != NULL)
	{
		m_incomingCallback(packet.m_payload, packet.m_payloadSize);
	}
	else
	{
		// Clean up if there is no one to hand the data to.
		delete packet.m_payload;
	}
}

void Xbee::processPacket(std::vector<XbeePacket>& packets)
{
	// No one to hand packets to, so cleanup
	if(m_incomingCallback == NULL)
	{
		for(auto p : packets)
		{
			delete p.m_payload;
		}
	}
	else
	{
		uint8_t payloadSize = 0;

		for(auto p : packets)
		{
			payloadSize += p.m_payloadSize;
		}

		uint8_t* finalPayload = new uint8_t[payloadSize];

		uint8_t* copyPtr = finalPayload;
		for(auto p : packets)
		{
			memcpy(copyPtr, p.m_payload, p.m_payloadSize);
			copyPtr += p.m_payloadSize;
			delete p.m_payload;
		}

		m_incomingCallback(finalPayload, payloadSize);
	}
}

