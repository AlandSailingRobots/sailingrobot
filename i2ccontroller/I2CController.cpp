#include "I2CController.h"
#include "SystemServices/Logger.h"
#include <unistd.h> // close

#include <wiringPiI2C.h>

std::mutex I2CController::m_mutex;


I2CController::I2CController()
	: m_Locked(false), m_DeviceFD(-1)
{

}

I2CController::~I2CController()
{
	close(m_DeviceFD);
}

bool I2CController::init(const int deviceAddress)
{
	m_DeviceFD = wiringPiI2CSetup(deviceAddress);

	return (m_DeviceFD < 0) ? false : true;
}

bool I2CController::write(uint8_t data)
{
	if (m_Locked)
	{
		return wiringPiI2CWrite(m_DeviceFD, data);
	}

	Logger::error("I2C controller transmission has not begun, call I2CController::beginTransmission!");
	return false;
}

bool I2CController::writeReg(uint8_t reg, uint8_t data)
{
	if (m_Locked)
	{
		return wiringPiI2CWriteReg8(m_DeviceFD, reg, data);
	}

	Logger::error("I2C controller transmission has not begun, call I2CController::beginTransmission!");
	return false;
}

int I2CController::read()
{
	if (m_Locked)
	{
		return wiringPiI2CRead(m_DeviceFD);
	}

	Logger::error("I2C controller transmission has not begun, call I2CController::beginTransmission!");
	return -1;
}

int I2CController::readReg(int regAddress)
{
	if (m_Locked)
	{
		return wiringPiI2CReadReg8(m_DeviceFD, regAddress);
	}

	Logger::error("I2C controller transmission has not begun, call I2CController::beginTransmission!");
	return -1;
}

int I2CController::readBlock(uint8_t* block, uint8_t size)
{
	if(m_Locked)
	{
		if(block == NULL)
		{
			Logger::error("%s char* block is a null pointer!", __PRETTY_FUNCTION__);
			return -1;
		}

		return wiringPiI2CReadBlock(m_DeviceFD, (char*)block, size);
	}

	Logger::error("I2C controller transmission has not begun, call I2CController::beginTransmission!");
	return -1;
}

/*
int I2CController::writeBlock(uint8_t* block, uint8_t size)
{
	if(m_Locked)
	{
		if(block == NULL)
		{
			Logger::error("%s char* block is a null pointer!", __PRETTY_FUNCTION__);
			return -1;
		}
		char command = 0;
		return wiringPiI2CWriteBlock(m_DeviceFD, command, (char*)block, size);
	}

	Logger::error("I2C controller transmission has not begun, call I2CController::beginTransmission!");
	return -1;
}
*/

void I2CController::beginTransmission()
{
	if(m_DeviceFD == -1)
	{
		Logger::error("%s Invalid device file descriptor, I2CController::init wasn't called or failed!", __PRETTY_FUNCTION__);
	}
	else
	{
		m_mutex.lock();
		m_Locked = true;
	}
}

void I2CController::endTransmission()
{
	m_Locked = false;
	m_mutex.unlock();
}
