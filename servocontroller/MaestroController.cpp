#include "MaestroController.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <mutex>



MaestroController::MaestroController() {
	m_ioDeviceHandle = -1;
}

void MaestroController::setPort(std::string portName)
{
	m_mutex.lock();
	m_ioDeviceHandlePath = portName;
	m_mutex.unlock();
}

//
void MaestroController::writeCommand(unsigned char type, int channel = -1, int value = -1)
{
	// Each DATA byte can only transmit seven bits of information.
	unsigned char mask = 0x7F,
				  valuePart1 = value & mask,
				  valuePart2 = (value >> 7) & mask;
	
	unsigned char channelLSB = channel & 0xFF;

	unsigned char command[] = {type, channelLSB, valuePart1, valuePart2};

	if (isOpenPort() == false)
	{
		openPort();
	}

	if (write(m_ioDeviceHandle, command, sizeof (command)) == -1)
	{
		throw "MaestroController::writeCommand()";
	}
}

int MaestroController::readRespons()
{
	if (isOpenPort() == false)
	{
		openPort();
	}

	unsigned short dataHandle = 0;
	if (read(m_ioDeviceHandle, &dataHandle, sizeof (dataHandle)) != sizeof (dataHandle))
	{
		throw "MaestroController::readRespons()";
	}
	return dataHandle;
}

int MaestroController::getError() {
	writeCommand(GET_ERROR, -1, -1);
	return readRespons();
}

void MaestroController::openPort()
{
	m_mutex.lock();
	m_ioDeviceHandle = open(m_ioDeviceHandlePath.c_str(), O_RDWR | O_NOCTTY);
	m_mutex.unlock();

	struct termios options;
	tcgetattr(m_ioDeviceHandle, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(m_ioDeviceHandle, TCSANOW, &options);

	if (isOpenPort() == false)
	{
		throw "MaestroController::openPort()";
	}
}

bool MaestroController::isOpenPort()
{
	if (m_ioDeviceHandle == -1)
	{
		return false;
	}
	return true;
}

MaestroController::~MaestroController()
{
	if (isOpenPort())
	{
		close(m_ioDeviceHandle);
	}
}
