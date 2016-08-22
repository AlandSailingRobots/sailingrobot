#include "WindowsSerialDataLink.h"
#include <iostream>

WindowsSerialDataLink::WindowsSerialDataLink(LPCSTR port, uint16_t baudRate)
: DataLink(true), m_port(port), m_baudRate(baudRate)
{

}

WindowsSerialDataLink::~WindowsSerialDataLink()
{
    CloseHandle(m_hSerial);
}

bool WindowsSerialDataLink::initialise(uint16_t frameSize)
{
	DataLink::initialise(frameSize);
	
	m_initialised = false;
	
	printf("Opening serial port...");
	m_hSerial = CreateFile(m_port,
							GENERIC_READ|GENERIC_WRITE,     // access ( read and write)
							0,                              // (share) 0:cannot share the COM port 
							0,                              // security  (None)    
							OPEN_EXISTING,                  // creation : open_existing
							FILE_ATTRIBUTE_NORMAL,           // we want overlapped operation
							NULL);                          // no templates file for COM port...

	if(m_hSerial == INVALID_HANDLE_VALUE)
	{
			printf("Error\n");
			return m_initialised;;
	}
	else
	{
			printf("OK\n");

			 // Set device parameters (baud, 1 start bit,
			// 1 stop bit, no parity)
			m_dcbSerialParam.DCBlength = sizeof(m_dcbSerialParam);
			if (GetCommState(m_hSerial, &m_dcbSerialParam) == 0)
			{
					printf("Error getting device state\n");
					CloseHandle(m_hSerial);
					return m_initialised;;
			}

			m_dcbSerialParam.BaudRate = m_baudRate;
			m_dcbSerialParam.ByteSize = 8;
			m_dcbSerialParam.StopBits = ONESTOPBIT;
			m_dcbSerialParam.Parity = NOPARITY;
			if(SetCommState(m_hSerial, &m_dcbSerialParam) == 0)
			{
					printf("Error setting device parameters\n");
					CloseHandle(m_hSerial);
					return m_initialised;;
			}

			// Set COM port timeout settings
			m_timeouts.ReadIntervalTimeout = 0;
			m_timeouts.ReadTotalTimeoutConstant = 0;
			m_timeouts.ReadTotalTimeoutMultiplier = 0;
			m_timeouts.WriteTotalTimeoutConstant = 0;
			m_timeouts.WriteTotalTimeoutMultiplier = 0;
			if(SetCommTimeouts(m_hSerial, &m_timeouts) == 0)
			{
					printf("Error setting timeouts\n");
					CloseHandle(m_hSerial);
					return m_initialised;;
			}
	}
		
	m_initialised = true;
	return m_initialised;
}

std::string WindowsSerialDataLink::sendATCommand(std::string command, uint16_t responseSize)
{ return "Nothing here buddy";}


void WindowsSerialDataLink::writeData(const uint8_t* data, uint8_t size)
{}
void WindowsSerialDataLink::readData(uint8_t* data, uint8_t size)
{}

void WindowsSerialDataLink::writeByte(uint8_t data)
{
    DWORD byteswritten;

    WriteFile(m_hSerial, &data, 1, &byteswritten, NULL);
}

int WindowsSerialDataLink::readByte()
{
    DWORD dwRead;
    int retVal;
    BYTE byte;

		if(not ReadFile(m_hSerial, &byte, 1, &dwRead, 0))
		{
			return -1;
		}
		
		retVal = byte;
		return retVal;
}

bool WindowsSerialDataLink::dataAvailable()
{
		return true;
}
