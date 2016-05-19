#ifndef AR_UNO_h
#define AR_UNO_h

#include <stdint.h> // uint8_t
#include <vector>
#include "AnalogArduino.h"
#include "models/AnalogArduinoModel.h"


#define DEFAULT_I2C_ADDRESS_PRESSURE 0x07

#define STARTBYTE 0x0F
// AR_UNO Commands


class AR_UNO:public AnalogArduino {
public:
        AR_UNO();
	~AR_UNO();

	// setup for the connection between Raspberry Pi and the PressureSensor
	bool init();

	// returns m_pressure
	int getValue0();
  // returns m_rudder
	int getValue1();
  // returns m_sheet
	int getValue2();
  // returns m_battery
	int getValue3();

	// returns model
	AnalogArduinoModel getModel();
  void readValues();
private:
	// file descriptor
	int m_fd;
	uint8_t m_address;

	AnalogArduinoModel m_model;

	int m_pressure;


	uint8_t readAddress();
};

#endif
