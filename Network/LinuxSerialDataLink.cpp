/****************************************************************************************
 *
 * File:
 * 		LinuxSerialDataLink.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "LinuxSerialDataLink.h"
#include "SystemServices/Logger.h"
#include <wiringSerial.h>
#include <thread>
#include <cstring>
#include "utility/SysClock.h"


#define ENTER_CMD_MODE() 	writeData((uint8_t*)"+++", 3); std::this_thread::sleep_for(std::chrono::milliseconds(3000))
#define EXIT_CMD_MODE()		writeData((uint8_t*)"ATCN\r", 5)


LinuxSerialDataLink::LinuxSerialDataLink(std::string port, uint16_t baudRate)
	:DataLink(true), m_handle(-1), m_port(port), m_baudRate(baudRate)
{


}

LinuxSerialDataLink::~LinuxSerialDataLink()
{

}

bool LinuxSerialDataLink::initialise(uint16_t frameSize)
{
	m_initialised = false;

	m_handle = serialOpen(m_port.c_str(), m_baudRate);

	if(m_handle < 0)
	{
		Logger::error("Failed to acquire a device handle for the Xbee");
		return m_initialised;
	}

	std::string response = sendATCommand("AT\r", 2);

	if(response[0] == 'O' && response[1] == 'K')
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

std::string LinuxSerialDataLink::sendATCommand(std::string command, uint16_t responseSize)
{
	uint8_t* response = new uint8_t[responseSize];

	ENTER_CMD_MODE();
	writeData((uint8_t*)command.c_str(), command.size());
	readData(response, responseSize);
	EXIT_CMD_MODE();

	std::string strResponse((char*)response);
	delete response;

	return strResponse;
}

void LinuxSerialDataLink::writeData(const uint8_t* data, uint8_t size)
{
	for(uint8_t index = 0; index < size; index++)
	{
		serialPutchar(m_handle, (char)data[index]);
	}
}

// TODO: Make safe, what if there is no data???
void LinuxSerialDataLink::readData(uint8_t* data, uint8_t size)
{
	for(uint8_t bytesRead = 0; bytesRead < size; bytesRead++)
	{
		data[bytesRead] = serialGetchar(m_handle);
	}
}

int LinuxSerialDataLink::readByte()
{
	return serialGetchar(m_handle);
}

void LinuxSerialDataLink::writeByte(uint8_t byte)
{
	serialPutchar(m_handle, byte);
}

bool LinuxSerialDataLink::dataAvailable()
{
	return serialDataAvail(m_handle);
}
