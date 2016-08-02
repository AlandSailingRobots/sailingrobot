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
#include "utility/SysClock.h"


#define AT_COMMAND_MODE_WAIT	3000
#define XBEE_TRANSMIT_TIME		50

#define PACKET_OVERHEAD			4

#define PACKET_START			0xC0
#define PACKET_ESCAPE			0xDB
#define PACKET_ESCAPE_START		0xDC


// REDO< FIX!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define ENTER_CMD_MODE() 	writeData((uint8_t*)"+++", 3); std::this_thread::sleep_for(std::chrono::milliseconds(AT_COMMAND_MODE_WAIT))
#define EXIT_CMD_MODE()		writeData((uint8_t*)"ATCN\r", 5)


Xbee::Xbee(bool master)
	:m_master(master), m_handle(-1), m_initialised(false), m_currPacketID(0)
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

	int finalSize = size;
	// Find Slip and Escape characters
	for(unsigned int i = 0; i < size; i++)
	{
		uint8_t c = data[i];
		if(c == PACKET_START || c == PACKET_ESCAPE)
		{
			finalSize++;
		}
	}

	// Turn the data into slip data
	uint8_t* slipData = new uint8_t[size];
	unsigned int slipIndex = 0;

	for(unsigned int i = 0; i < size; i++)
	{
		uint8_t c = data[i];

		if(c == PACKET_START)
		{
			slipData[slipIndex++] = PACKET_ESCAPE;
			slipData[slipIndex++] = PACKET_ESCAPE_START;
		}
		else
		{
			slipData[slipIndex++] = c;
		}
	}

	// Split the packet
	if(size > maxDataSize)
	{
		// Round up
		uint8_t packetCount = 1 + ((size - 1) / maxDataSize);
		uint8_t* dataPtr = slipData;

		// Locks until the function returns and the current scope is left
		std::lock_guard<std::mutex> lock(m_transmitQueueMutex);

		for(uint8_t i = 0; i < packetCount - 1; i++)
		{
			XbeePacket packet;
			packet.m_packetID = m_currPacketID;
			packet.m_packetCount = packetCount;
			packet.m_payloadSize = size > maxDataSize ? maxDataSize : size;
			packet.m_payload = dataPtr;
			dataPtr = dataPtr + maxDataSize;
			size = size - maxDataSize;

			m_transmitQueue.push(packet);
		}
	}
	else
	{
		XbeePacket packet;
		packet.m_packetID = m_currPacketID;
		packet.m_packetCount = 1;
		packet.m_payloadSize = size;
		packet.m_payload = slipData;

		m_transmitQueue.push(packet);
	}

	m_currPacketID++;
}

void Xbee::processRadioMessages()
{
	static unsigned long int lastReceived = 0;
	const int TRANSMIT_WAIT = 1; // 1 seconds, should probably be less, like 500ms
	const int PACKETS_TO_TRANSMIT = 5;

	bool packetsReceived = receivePackets();
	processPacketQueue();

	if(packetsReceived)
	{
		lastReceived = SysClock::unixTime();
	}

	// Packets to transmit
	if(m_transmitQueue.size() > 0)
	{
		// Master gets priority otherwise we wait until TRANSMIT_WAIT time has past before the last read
		// before transmitting
		if(m_master)
		{
			// Locks until the function returns and the current scope is left
			std::lock_guard<std::mutex> lock(m_transmitQueueMutex);
			uint16_t msgsToTransmit = m_transmitQueue.size();

			for(uint16_t i = 0; i < msgsToTransmit; i++)
			{
				writeData(m_transmitQueue.front());
				m_transmitQueue.pop();
			}
		}
		// When we are not in master mode we should only transmit a few packets then break incase the master
		// wants to interrupt.
		else if((SysClock::unixTime() - lastReceived) > TRANSMIT_WAIT)
		{
			uint16_t msgsToTransmit = m_transmitQueue.size();

			for(uint16_t i = 0;(i < msgsToTransmit || i == PACKETS_TO_TRANSMIT); i++)
			{
				writeData(m_transmitQueue.front());
				m_transmitQueue.pop();
			}
		}
	}
}


void Xbee::writeData(XbeePacket packet)
{
	serialPutchar(m_handle, (char)PACKET_START);
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
	bool listen = true;
	bool foundEscape = false;
	int temp = 0;

	// Look for start character
	while(listen)
	{
		if(serialDataAvail(m_handle) > 0);
		{
			int c = serialGetchar(m_handle);

			if((uint8_t)c == PACKET_ESCAPE)
			{
				foundEscape = true;
			}
			else
			{
				foundEscape = false;
			}

			if((uint8_t)c == PACKET_START && not foundEscape)
			{
				listen = false;
			}
		}

		if(temp == 50000)
		{
			return 0;
		}
		temp++;
	}

	//const uint8_t maxDataSize = MAX_PACKET_SIZE - PACKET_OVERHEAD;
	//uint8_t tempBuffer[maxDataSize];

	// Get header packets
	packet.m_packetID = serialGetchar(m_handle);
	packet.m_packetCount = serialGetchar(m_handle);
	packet.m_payloadSize = serialGetchar(m_handle);
	if(packet.m_payloadSize == 0)
	{
		return 0;
	}
	packet.m_payload = new uint8_t[packet.m_payloadSize];

	uint8_t bytesRead = 0;

	for(unsigned int i = 0; i < packet.m_payloadSize; i++)
	{
		int c = serialGetchar(m_handle);

		if(c != -1)
		{
			packet.m_payload[i] = c;
			bytesRead++;
		}
	}

	// Replace with check sum
	if(bytesRead != packet.m_payloadSize)
	{
		delete packet.m_payload;
		bytesRead = 0;
	}

	/*int bytesAvailable = serialDataAvail(m_handle);
	uint8_t bytesRead = 0;

	// The packet must have more data than the packet overhead
	if(bytesAvailable > PACKET_OVERHEAD)
	{
		bytesAvailable = bytesAvailable - 3;
		packet.m_packetID = serialGetchar(m_handle);
		packet.m_packetCount = serialGetchar(m_handle);
		packet.m_payloadSize = serialGetchar(m_handle);

		if(packet.m_payloadSize > MAX_PACKET_SIZE || packet.m_payloadSize == 0)
		{
			return 0;
		}

		while(packet.m_payloadSize > serialDataAvail(m_handle)) { }

		// Check all the payload made it
		if(bytesAvailable >= packet.m_payloadSize)
		{
			Logger::info("Got packet %d", packet.m_payloadSize);
			packet.m_payload = new uint8_t[packet.m_payloadSize];

			Xbee::readData(packet.m_payload, packet.m_payloadSize);
			bytesRead = packet.m_payloadSize + PACKET_OVERHEAD;;
		}
		else
		{
			//Logger::error("%s Malformed packet - Less bytes available than the packet payload, expected %d, got %d", __PRETTY_FUNCTION__, packet.m_payloadSize, bytesAvailable);
		}

	}
	else
	{
		//Logger::warning("%s No Data - Failed to receive data from the Xbee", __PRETTY_FUNCTION__);
	}*/

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

bool Xbee::receivePackets()
{
	// Check for packets to be available and if there isn't anything to do with
	// the packets, don't bother receiving them
	if(not dataAvailable() || m_incomingCallback == NULL)
	{
		return false;
	}

	bool readPackets = true;
	int packetsReceived = 0;

	// TODO - Jordan: Add a time check here too just in case
	while(readPackets)
	{
		XbeePacket packet;

		if(readData(packet) && packet.m_payloadSize > 0)
		{
			m_receiveQueue.push(packet);
			m_packetsReceived++;
			packetsReceived++;
		}
		else
		{
			readPackets = false;
		}
	}

	return (packetsReceived > 0);
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
		Logger::info("Single inbound packet Size: %d", packet.m_payloadSize);

		// Deslip it

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
	if(m_incomingCallback == NULL || packets.size() != packets[0].m_packetCount)
	{
		// The first packet in the multi-pack actually holds the valid pointer.
		delete packets[0].m_payload;
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
		}

		delete packets[0].m_payload;

		Logger::info("Multi-packet Size: %d", payloadSize);
		m_incomingCallback(finalPayload, payloadSize);
	}
}

