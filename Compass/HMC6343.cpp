#include "HMC6343.h"
#include <wiringPiI2C.h>
#include <wiringPi.h> // delay
#include <unistd.h> // close
#include <vector>
#include "utility/Utility.h"


HMC6343::HMC6343(unsigned int headningBufferSize) :
	m_model(CompassModel(0,0,0,AccelerationModel(0,0,0) )),
	m_headningBufferSize(headningBufferSize)
{
	m_address = DEFAULT_I2C_ADDRESS;
	m_fd = -1;

	m_magX = 0;
	m_magY = 0;
	m_magZ = 0;
}

HMC6343::~HMC6343()
{
	// behöver man lägga den i sleep/standby?
	close(m_fd);
}

bool HMC6343::init()
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
		data = readEEPROM(REG_SLAVE_ADDRESS);
		
		// Check if value read in EEPROM is the expected value for the HMC6343 I2C address
		if (data != DEFAULT_I2C_ADDRESS)	{
			success = false; // Init failed, EEPROM did not read expected I2C address value
		}
	}

	return success;
}

int HMC6343::getHeading()
{
	return m_model.heading;
}

int HMC6343::getPitch()
{
	return m_model.pitch;
}

int HMC6343::getRoll()
{
	return m_model.roll;
}

int HMC6343::getAccelX()
{
	return m_model.accelerationModel.accelX;
}

int HMC6343::getAccelY()
{
	return m_model.accelerationModel.accelY;
}

int HMC6343::getAccelZ()
{
	return m_model.accelerationModel.accelZ;
}

void HMC6343::readValues()
{
	std::vector<uint8_t> headingVector = readGeneric(COM_POST_HEADING);

	m_headings.push_back(
		(Utility::combineBytes(headingVector.at(0), headingVector.at(1))) / 10.0);
	while (m_headings.size() > m_headningBufferSize)
	{
		m_headings.erase(m_headings.begin());
	}

	m_model.heading = int(Utility::meanOfAngles(m_headings) + 0.5);
	m_model.pitch = (Utility::combineBytesSigned(headingVector.at(2), headingVector.at(3))) / 10;
	m_model.roll = (Utility::combineBytesSigned(headingVector.at(4), headingVector.at(5))) / 10;
}

void HMC6343::readMag()
{
	std::vector<uint8_t> magVector = readGeneric(COM_POST_MAG);

	m_magX = Utility::combineBytes(magVector.at(0), magVector.at(1));
	m_magY = Utility::combineBytes(magVector.at(2), magVector.at(3));
	m_magZ = Utility::combineBytes(magVector.at(4), magVector.at(5));
}

void HMC6343::readTilt()
{
	std::vector<uint8_t> tiltVector = readGeneric(COM_POST_TILT);

	m_model.pitch = Utility::combineBytes(tiltVector.at(0), tiltVector.at(1));
	m_model.roll = Utility::combineBytes(tiltVector.at(2), tiltVector.at(3));
	m_temperature = Utility::combineBytes(tiltVector.at(4), tiltVector.at(5));
}

void HMC6343::readAccel()
{
	std::vector<uint8_t> accelVector = readGeneric(COM_POST_ACCEL);

	m_model.accelerationModel.accelX = Utility::combineBytesSigned(accelVector.at(0), accelVector.at(1));
	m_model.accelerationModel.accelY = Utility::combineBytesSigned(accelVector.at(2), accelVector.at(3));
	m_model.accelerationModel.accelZ = Utility::combineBytesSigned(accelVector.at(4), accelVector.at(5));
}

std::vector<uint8_t> HMC6343::readGeneric(uint8_t command){

	const int transferSize = 6;
	std::vector<uint8_t> returnVector;
	// head(MSB/LSB), pitch(MSB/LSB), roll(MSB/LSB)

	wiringPiI2CWrite(m_fd, command);
	delay(1); // wait for processing of command

	for(int i = 0; i < transferSize; i++) {
		returnVector.push_back( wiringPiI2CRead(m_fd) );
		
	}
        delay(1);
	return returnVector;
}

void HMC6343::setOrientation(uint8_t orientation)
{
	if (orientation == LEVEL)
		{
			wiringPiI2CWrite(m_fd, COM_ORIENT_LEVEL);
		}
	else if (orientation == SIDEWAYS)
		{
			wiringPiI2CWrite(m_fd, COM_ORIENT_SIDEWAYS);
		}
	else if (orientation == FLATFRONT)
		{
			wiringPiI2CWrite(m_fd, COM_ORIENT_FLATFRONT);
		}
}


// Send a command to the HMC6343 to read a specified register of the EEPROM
uint8_t HMC6343::readEEPROM(uint8_t reg)
{
	uint8_t data = 0x00;
	wiringPiI2CWrite(m_fd, COM_READ_EEPROM);
	wiringPiI2CWrite(m_fd, reg);
	delay(10); // wait for processing of command

	data = wiringPiI2CRead(m_fd);

	return data;
}

CompassModel HMC6343::getModel() {
	return m_model;
}
