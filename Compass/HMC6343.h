#ifndef HMC6343_h
#define HMC6343_h

#include <stdint.h> // uint8_t
#include <vector>
#include "Compass.h"
#include "models/CompassModel.h"


#define DEFAULT_I2C_ADDRESS 0x19

// HMC6343 Registers
#define REG_SLAVE_ADDRESS 0x00

// HMC6343 Commands
#define COM_POST_HEADING 0x50
#define COM_POST_TILT 0x55
#define COM_POST_MAG 0x45
#define COM_POST_ACCEL 0x40
#define COM_READ_EEPROM 0xE1


#define COM_ORIENT_LEVEL 0x72
#define COM_ORIENT_SIDEWAYS 0x73
#define COM_ORIENT_FLATFRONT 0x74

// HMC6343 Orientations
#define LEVEL 0 // X = forward, +Z = up (default)
#define SIDEWAYS 1 // X = forward, +Y = up
#define FLATFRONT 2 // Z = forward, -X = up

class HMC6343:public Compass{
public:
	HMC6343(unsigned int headningBufferSize);
	~HMC6343();

	// setup for the connection between Raspberry Pi and the compass
	bool init();

	// returns m_heading
	int getHeading();

	// returns m_heading
	int getPitch();

	// returns m_heading
	int getRoll();

	// returns accelX
	int getAccelX();
        
        // returns accelY
	int getAccelY();
        
        // returns accelZ
	int getAccelZ();

	// read heading, pitch and roll
	void readValues();

	// read tilt data
	void readTilt();

	// read magnetometer values
	void readMag();

	// read accelerometer values
	void readAccel();

	// set orientation data for correct readings
	void setOrientation(uint8_t orientation);

	// returns model
	CompassModel getModel();

private:
	// file descriptor
	int m_fd;
	uint8_t m_address;

	CompassModel m_model;

	int m_temperature;
	int m_magX;
	int m_magY;
	int m_magZ;

	const unsigned int m_headningBufferSize;
	std::vector<float> m_headings;


	// read a specified register of the EEPROM
	uint8_t readEEPROM(uint8_t reg);


	std::vector<uint8_t> readGeneric(uint8_t command);
};

#endif
