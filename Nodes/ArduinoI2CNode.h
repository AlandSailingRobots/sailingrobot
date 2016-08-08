/****************************************************************************************
 *
 * File:
 * 		ArduinoI2CNode.h
 *
 * Purpose:
 *		The Arduino node communicates with the arduino. Sends data about the pressure, rudder, sheet and battery.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"
#include "i2ccontroller/I2CController.h"
#include <mutex>



class ArduinoI2CNode : public ActiveNode {
public:
	ArduinoI2CNode(MessageBus& msgBus);

	///----------------------------------------------------------------------------------
	/// Attempts to connect to the Arduino.
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// This function should be used to start the active nodes thread.
 	///----------------------------------------------------------------------------------
	void start();

	void processMessage(const Message* msg);

	int sendToArduino(uint8_t* data, uint8_t dataID);

	void passToCANBus();

	void processI2CData(uint8_t* data);

private:
	static void ArduinoThreadFunc(void* nodePtr);

	I2CController 	m_I2C;
	bool 			m_Initialised;
	bool 			m_locked;
	std::mutex 		m_mutex;

	//adding instead of replacing for testing purposes
	uint16_t m_rudderCommand;
	uint16_t m_sailCommand;
	uint8_t m_windVaneCommand;
	bool m_actuatorCommandsReady;
	bool m_windVaneCommandReady;
};
