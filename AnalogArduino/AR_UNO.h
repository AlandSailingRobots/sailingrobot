#ifndef AR_UNO_h
#define AR_UNO_h

#include <stdint.h> // uint8_t
#include <vector>
#include "AnalogArduino.h"
#include "models/AnalogArduinoModel.h"


#define DEFAULT_I2C_ADDRESS_PRESSURE 0x07

#define STARTBYTE 0x0F
// AR_UNO Commands

#define COM_READ_PRESSURE 0x01
#define COM_READ_RUDDER 0x02
#define COM_READ_SHEET 0x03
#define COM_READ_BATTERY 0x04
#define COM_READ_ADDR 0x22



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
  void readPressure();
  void readRudder();
  void readSheet();
  void readBattery();
  void readValues();
private:
	// file descriptor
	int m_fd;
	uint8_t m_address;

	AnalogArduinoModel m_model;

	int m_pressure;

	uint16_t readAnalog(uint8_t command);

	uint8_t readAddress();
};

#endif
