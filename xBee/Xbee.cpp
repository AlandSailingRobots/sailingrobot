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

#define PACKET_OVERHEAD			6 // START character(1), Packet ID(1), packet count(1), payload size(1), checksum(2)

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

	uint16_t slipSize = 0;

	// SLIP the data
	uint8_t* slipData = slip(data, size, slipSize);

	// Split the data if its bigger than one packet
	if(slipSize > maxDataSize)
	{
		// Round up
		uint8_t packetCount = 1 + ((slipSize - 1) / maxDataSize);
		uint8_t* dataPtr = slipData;

		// Locks until the function returns and the current scope is left
		std::lock_guard<std::mutex> lock(m_transmitQueueMutex);

		for(uint8_t i = 0; i < packetCount; i++)
		{
			XbeePacket packet;
			packet.m_packetID = m_currPacketID;
			packet.m_packetCount = packetCount;
			packet.m_payloadSize = slipSize > maxDataSize ? maxDataSize : slipSize;
			packet.m_payload = dataPtr;
			packet.m_checksum = fletcherChecksum(packet.m_payload, packet.m_payloadSize);

			dataPtr = dataPtr + packet.m_payloadSize;
			slipSize = slipSize - packet.m_payloadSize;

			m_transmitQueue.push(packet);
		}
	}
	// Fits into a single packet
	else
	{
		XbeePacket packet;
		packet.m_packetID = m_currPacketID;
		packet.m_packetCount = 1;
		packet.m_payloadSize = slipSize;
		packet.m_payload = slipData;
		packet.m_checksum = fletcherChecksum(packet.m_payload, packet.m_payloadSize);

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

bool Xbee::receivePackets()
{
	const uint8_t MAX_PACKETS_TO_RECEIVE = 10;

	// Check for packets to be available and if there isn't anything to do with
	// the packets, don't bother receiving them
	if(not dataAvailable() || m_incomingCallback == NULL)
	{
		return false;
	}

	bool readPackets = true;
	int packetsReceived = 0;

	while(readPackets || packetsReceived == MAX_PACKETS_TO_RECEIVE)
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
		//Logger::info("Single inbound packet Size: %d", packet.m_payloadSize);

		// Deslip it
		uint16_t dataSize = 0;
		uint8_t* dataPtr = deslip(packet.m_payload, packet.m_payloadSize, dataSize);

		// Clean up the packet
		delete packet.m_payload;

		m_incomingCallback(dataPtr, dataSize);
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

		uint16_t dataSize = 0;
		uint8_t* dataPtr = deslip(finalPayload, payloadSize, dataSize);

		m_incomingCallback(dataPtr, dataSize);
	}

	// Clean up the packets
	for(auto p : packets)
	{
		delete p.m_payload;
	}

	packets.clear();
}

void Xbee::writeData(XbeePacket packet)
{
	serialPutchar(m_handle, (char)PACKET_START);
	serialPutchar(m_handle, (char)packet.m_packetID);
	serialPutchar(m_handle, (char)packet.m_packetCount);
	serialPutchar(m_handle, (char)packet.m_payloadSize);

	writeData(packet.m_payload, packet.m_payloadSize);
	serialPutchar(m_handle, (char)packet.m_checksum);
	serialPutchar(m_handle, (char)(packet.m_checksum >> 8));

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
		if(serialDataAvail(m_handle) > 0)
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
		else
		{
			temp++;
		}

		if(temp == 100)
		{
			return 0;
		}
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

	uint8_t checksum1 = serialGetchar(m_handle);
	uint8_t checksum2 = serialGetchar(m_handle);

	packet.m_checksum = (checksum2 << 8) | checksum1;

	if(bytesRead != packet.m_payloadSize)
	{
		delete packet.m_payload;
		bytesRead = 0;
	}

	if(packet.m_checksum != fletcherChecksum(packet.m_payload, packet.m_payloadSize))
	{
		delete packet.m_payload;
		bytesRead = 0;
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

uint8_t* Xbee::slip(uint8_t* data, uint16_t size, uint16_t& slipSize)
{
	slipSize = size;

	// Find Slip and Escape characters
	for(unsigned int i = 0; i < size; i++)
	{
		uint8_t c = data[i];
		if(c == PACKET_START || c == PACKET_ESCAPE)
		{
			slipSize++;
		}
	}

	// Turn the data into slip data
	uint8_t* slipDataPtr = new uint8_t[slipSize];
	unsigned int slipIndex = 0;

	for(unsigned int i = 0; i < size; i++)
	{
		uint8_t c = data[i];

		if(c == PACKET_START)
		{
			slipDataPtr[slipIndex++] = PACKET_ESCAPE;
			slipDataPtr[slipIndex++] = PACKET_ESCAPE_START;
		}
		else
		{
			slipDataPtr[slipIndex++] = c;
		}
	}

	return slipDataPtr;
}


uint8_t* Xbee::deslip(uint8_t* slipData, uint16_t slipSize, uint16_t& size)
{
	size = slipSize;

	// Find Slip and Escape characters
	for(int i = 0; i < (slipSize - 1); i++)
	{
		uint8_t c = slipData[i];
		if(c == PACKET_ESCAPE && slipData[i + 1] == PACKET_START)
		{
			size--;
		}
	}

	// Turn the slip data into normal data
	uint8_t* data = new uint8_t[size];
	unsigned int index = 0;

	bool escapeFound = false;

	for(int i = 0; i < (slipSize - 1); i++)
	{
		uint8_t c = slipData[i];

		if(escapeFound && c == PACKET_ESCAPE_START)
		{
			data[index++] = PACKET_START;
			escapeFound = false;
		}
		else if(c == PACKET_ESCAPE)
		{
			escapeFound = true;
		}
		else
		{
			escapeFound = false;
			data[index++] = c;
		}
	}

	return data;
}

uint16_t Xbee::fletcherChecksum(uint8_t* data, uint16_t size)
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
