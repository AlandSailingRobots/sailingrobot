#include "MaestroController.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>



MaestroController::MaestroController() {
	ioDeviceHandle = -1;
}

void MaestroController::setPort(std::string portName)
{
	ioDeviceHandlePath = portName;
}

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

	if (write(ioDeviceHandle, command, sizeof (command)) == -1)
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
	if (read(ioDeviceHandle, &dataHandle, sizeof (dataHandle)) != sizeof (dataHandle))
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
	ioDeviceHandle = open(ioDeviceHandlePath.c_str(), O_RDWR | O_NOCTTY);

	struct termios options;
	tcgetattr(ioDeviceHandle, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(ioDeviceHandle, TCSANOW, &options);

	if (isOpenPort() == false)
	{
		throw "MaestroController::openPort()";
	}
}

bool MaestroController::isOpenPort()
{
	if (ioDeviceHandle == -1)
	{
		return false;
	}
	return true;
}

MaestroController::~MaestroController()
{
	if (isOpenPort())
	{
		close(ioDeviceHandle);
	}
}
