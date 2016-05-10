#ifndef PTMN_STS_h
#define PTMN_STS_h

#include <stdint.h> // uint8_t
#include <vector>
#include "PressureSensor.h"
#include "models/PressureSensorModel.h"


#define DEFAULT_I2C_ADDRESS 0x07

#define STARTBYTE 0x0F
// PTMN_STS Commands

#define COM_READ_PRESSURE 0x22
#define COM_READ_ADDR 0x33



class PTMN_STS:public PressureSensor{
public:
        PTMN_STS();
	~PTMN_STS();

	// setup for the connection between Raspberry Pi and the PressureSensor
	bool init();

	// returns m_pressure
	int getPressure();

	// returns model
	PressureSensorModel getModel();

private:
	// file descriptor
	int m_fd;
	uint8_t m_address;

	PressureSensorModel m_model;

	int m_pressure;

	uint16_t readPressure();
	uint8_t readAddress();
};

#endif
