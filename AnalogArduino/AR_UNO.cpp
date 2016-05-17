#include "AR_UNO.h"
#include <wiringPiI2C.h>
#include <wiringPi.h> // delay
#include <unistd.h> // close
#include <vector>
#include "utility/Utility.h"


AR_UNO::AR_UNO():
	m_model(AnalogArduinoModel(0,0,0,0))
{
	m_address = DEFAULT_I2C_ADDRESS_PRESSURE;
	m_fd = -1;
}

AR_UNO::~AR_UNO()
{
	close(m_fd);
}

bool AR_UNO::init()
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

		// Check if value read in EEPROM is the expected value for the AR_UNO I2C address
		if (data != DEFAULT_I2C_ADDRESS_PRESSURE)	{
			success = false; // Init failed, EEPROM did not read expected I2C address value
		}
	}

	return success;
}

int AR_UNO::getValue0()
{
	return m_model.analogValue0;
}

int AR_UNO::getValue1()
{
	return m_model.analogValue1;
}

int AR_UNO::getValue2()
{
	return m_model.analogValue2;
}

int AR_UNO::getValue3()
{
	return m_model.analogValue3;
}


void AR_UNO::readPressure()
{
  m_model.analogValue0 = readAnalog(COM_READ_PRESSURE);
}

void AR_UNO::readRudder()
{
	m_model.analogValue1 = readAnalog(COM_READ_RUDDER);
}

void AR_UNO::readSheet()
{
	m_model.analogValue2 = readAnalog(COM_READ_SHEET);
}

void AR_UNO::readBattery()
{
	m_model.analogValue3 = readAnalog(COM_READ_BATTERY);
}

void AR_UNO::readValues()
{
	readPressure();
	readSheet();
	readRudder();
	readBattery();
}

uint16_t AR_UNO::readAnalog(uint8_t command){

	const int transferSize = 2;
	std::vector<uint8_t> tempVector;
	// DATA (MSB/LSB)

	wiringPiI2CWrite(m_fd, STARTBYTE);
	delay(1); // wait for processing of command
	wiringPiI2CWrite(m_fd, command);
	delay(1); // wait for processing of command

	for(int i = 0; i < transferSize; i++) {
		tempVector.push_back( wiringPiI2CRead(m_fd) );

	}
	delay(1);
	uint16_t returnVal = tempVector[0]<<8;
	returnVal += (uint16_t) tempVector[1];
	return returnVal;
}

uint8_t AR_UNO::readAddress(){


	wiringPiI2CWrite(m_fd, STARTBYTE);
	delay(1); // wait for processing of command
	wiringPiI2CWrite(m_fd, COM_READ_ADDR);
	delay(1); // wait for processing of command

	return wiringPiI2CRead(m_fd);
}

AnalogArduinoModel AR_UNO::getModel() {
	return m_model;
}
