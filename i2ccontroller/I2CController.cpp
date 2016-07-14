#include "I2CController.h"
#include <unistd.h> // close

std::mutex I2CController::m_mutex;


I2CController::I2CController(int fd) : m_fd(fd)
{
	locked = false;
}

I2CController::~I2CController()
{
	close(m_fd);
}

int I2CController::setup(int address)
{
	if (locked)
	{
		return wiringPiI2CSetup(address);
	}

	return 0; //This should never happen unless you are silly. beginTransmission needs to be called before this function
}

std::vector<uint8_t> I2CController::readGeneric(uint8_t command){

	if(locked)
	{
		const int transferSize = 6;
		std::vector<uint8_t> returnVector;
		// head(MSB/LSB), pitch(MSB/LSB), roll(MSB/LSB)

		wiringPiI2CWrite(m_fd, command);
		delay(1); // wait for processing of command

		for (int i = 0; i < transferSize; i++)
		{
			returnVector.push_back(wiringPiI2CRead(m_fd));
		}
		delay(1);

		return returnVector;
	}

	return {}; //This should never happen unless you are silly. beginTransmission needs to be called before this function
}

int I2CController::write(int data)
{
	if (locked)
	{
		return wiringPiI2CWrite(m_fd, data);
	}

	return 0; //This should never happen unless you are silly. beginTransmission needs to be called before this function
}

int I2CController::read()
{
	if (locked)
	{
		return wiringPiI2CRead(m_fd);
	}

	return 0; //This should never happen unless you are silly. beginTransmission needs to be called before this function
}

int I2CController::readBlock()
{
	if(locked)
	{
		uint8_t block[40];
		wiringPiI2CReadBlock(m_fd, block);

		return block[10];
	}

	return 0; //This should never happen unless you are silly. beginTransmission needs to be called before this function
}


void I2CController::beginTransmission()
{
	m_mutex.lock();
	locked = true;
}

void I2CController::endTransmission()
{
	locked = false;
	m_mutex.unlock();
}
