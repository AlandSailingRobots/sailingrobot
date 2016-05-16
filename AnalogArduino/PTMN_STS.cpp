#include "PTMN_STS.h"
#include <wiringPiI2C.h>
#include <wiringPi.h> // delay
#include <unistd.h> // close
#include <vector>
#include "utility/Utility.h"


PTMN_STS::PTMN_STS():
	m_model(AnalogArduinoModel(0))
{
	m_address = DEFAULT_I2C_ADDRESS_PRESSURE;
	m_fd = -1;
}

PTMN_STS::~PTMN_STS()
{
	close(m_fd);
}

bool PTMN_STS::init()
{
	bool success = true;
	uint8_t data = 0x00;

	// set up I2C
	m_fd = wiringPiI2CSetup(m_address);

	if (m_fd == -1) {
		success = false; // you can consult errno?
	}
	else {
		// Check for device by reading I2C address from EEPROM
		data = readAddress();

		// Check if value read in EEPROM is the expected value for the PTMN_STS I2C address
		if (data != DEFAULT_I2C_ADDRESS_PRESSURE)	{
			success = false; // Init failed, EEPROM did not read expected I2C address value
		}
	}

	return success;
}

int PTMN_STS::getPressure()
{
	m_model.analogValue = readPressure();
	return m_model.analogValue;
}

 uint16_t PTMN_STS::readPressure(){

	const int transferSize = 2;
	std::vector<uint8_t> tempVector;
	// PRESSURE(MSB/LSB)

	wiringPiI2CWrite(m_fd, STARTBYTE);
	delay(1); // wait for processing of command
	wiringPiI2CWrite(m_fd, COM_READ_PRESSURE);
	delay(1); // wait for processing of command

	for(int i = 0; i < transferSize; i++) {
		tempVector.push_back( wiringPiI2CRead(m_fd) );

	}
	uint16_t returnVal = tempVector[0]<<8;
	returnVal += (uint16_t) tempVector[1];
	return returnVal;
}

uint8_t PTMN_STS::readAddress(){


	wiringPiI2CWrite(m_fd, STARTBYTE);
	delay(1); // wait for processing of command
	wiringPiI2CWrite(m_fd, COM_READ_ADDR);
	delay(1); // wait for processing of command

	return wiringPiI2CRead(m_fd);
}

AnalogArduinoModel PTMN_STS::getModel() {
	return m_model;
}
