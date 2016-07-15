#include "AR_UNO.h"
#include <wiringPiI2C.h>
#include <wiringPi.h> // delay
#include <unistd.h> // close
#include <vector>
#include <cstring>
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

void AR_UNO::readValues()
{
  uint8_t block[40];
  uint16_t reVal;
  memset(block,0xFF,40);
  wiringPiI2CReadBlock(m_fd, (char*)block, 10);

  reVal = block[2]<<8;
  reVal+=(uint16_t) block[3];
  m_model.analogValue0 = reVal;
  reVal = block[4]<<8;
  reVal+=(uint16_t) block[5];
  m_model.analogValue1 = reVal;
  reVal = block[6]<<8;
  reVal+=(uint16_t) block[7];
  m_model.analogValue2 = reVal;
  reVal = block[8]<<8;
  reVal+=(uint16_t) block[9];
  m_model.analogValue3 = reVal;
}

uint8_t AR_UNO::readAddress(){
	uint8_t block[40];
	wiringPiI2CReadBlock(m_fd,(char*)block, 11);
	return block[9];
}


AnalogArduinoModel AR_UNO::getModel() {
	return m_model;
}
