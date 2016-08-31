/****************************************************************************************
 *
 * File:
 * 		MaestroController.cpp
 *
 * Purpose:
 *		Provides an interface to the USB MaestroController that controls the vessel's
 *		motors. This class is thread safe.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "MaestroController.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "SystemServices/Logger.h"


std::mutex 	MaestroController::m_Mutex;
int MaestroController::m_Handle = -1;


bool MaestroController::init(std::string portName)
{
	m_Handle = open(portName.c_str(), O_RDWR | O_NOCTTY);

	struct termios options;
	tcgetattr(m_Handle, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(m_Handle, TCSANOW, &options);

	return (m_Handle < 0) ? false : true;
}

bool MaestroController::writeCommand(MaestroCommands cmd, int channel, int value)
{
	bool success = true;

	if(m_Handle > -1)
	{
		// Locks until the function returns and the current scope is left
		std::lock_guard<std::mutex> lock(m_Mutex);

		// Each DATA byte can only transmit seven bits of information.
		uint8_t mask = 0x7F;
		uint8_t valuePart1 = value & mask;
		uint8_t valuePart2 = (value >> 7) & mask;

		uint8_t channelLSB = channel & 0xFF;

		// Package up the command packet
		uint8_t command[] = {(uint8_t)cmd, channelLSB, valuePart1, valuePart2};

		if (write(m_Handle, command, sizeof (command)) == -1)
		{
			Logger::error("%s Failed to write command to MaestroController", __PRETTY_FUNCTION__);
			success = false;
		}
	}
	else
	{
		Logger::error("%s Handle is not valid, was Maestro Controller::init called, or did it fail?", __PRETTY_FUNCTION__);
		success = false;
	}

	return success;
}

int MaestroController::readResponse()
{
	uint8_t buff[2];
	int response = -1;

	if(m_Handle > -1)
	{
		// Locks until the function returns and the current scope is left
		std::lock_guard<std::mutex> lock(m_Mutex);

		if (read(m_Handle, &buff, sizeof (buff)) == sizeof (buff))
		{
			response = (buff[0] <<8) | buff[1];
		}
		else
		{
			Logger::error("%s Failed to read data from the Maestro Controller", __PRETTY_FUNCTION__);
		}
	}
	else
	{
		Logger::error("%s Handle is not valid, was MaestroController::init called, or did it fail?", __PRETTY_FUNCTION__);
	}

	return response;
}

int MaestroController::readCommand(MaestroCommands cmd, int channel)
{
	uint8_t buff[2];
	int response = -1;

	if(m_Handle > -1)
	{
		// Locks until the function returns and the current scope is left
		std::lock_guard<std::mutex> lock(m_Mutex);

		uint8_t channelLSB = channel & 0xFF;

		// Package up the command packet
		uint8_t command[] = {(uint8_t)cmd, channelLSB};

		if (write(m_Handle, command, sizeof (command)) == -1)
		{
			Logger::error("%s Failed to write command to MaestroController", __PRETTY_FUNCTION__);
		}
		else
		{
			if (read(m_Handle, &buff, sizeof (buff)) == sizeof (buff))
			{
				response = (buff[0] <<8) | buff[1];
			}
			else
			{
				Logger::error("%s Failed to read data from the Maestro Controller", __PRETTY_FUNCTION__);
			}
		}
	}
	else
	{
		Logger::error("%s Handle is not valid, was Maestro Controller::init called, or did it fail?", __PRETTY_FUNCTION__);
	}

	return response;
}

int MaestroController::getError()
{
	writeCommand(MaestroCommands::GetError, -1, -1);
	return readResponse();
}
