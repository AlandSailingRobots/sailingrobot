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

	///----------------------------------------------------------------------------------
 	/// Parses subscrubed messages
 	///----------------------------------------------------------------------------------
	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
 	/// Sends messages to the arduino over I2C
 	///----------------------------------------------------------------------------------
	int sendToArduino(uint8_t* data, uint8_t dataID);

	///----------------------------------------------------------------------------------
 	/// Takes a system message and converts it into the format of a CANBus message, ready
	/// to be sent to the Arduino over I2C, then passes this data to sendToArduino
 	///----------------------------------------------------------------------------------
	void passToCANBus();

	///----------------------------------------------------------------------------------
 	/// Parses the incoming I2C data and publishes respective messages
 	///----------------------------------------------------------------------------------
	void processI2CData(uint8_t* data);

private:
	static void ArduinoThreadFunc(void* nodePtr);

	I2CController 	m_I2C;
	bool 			m_Initialised;
	bool 			m_locked;
	std::mutex 		m_mutex;
	uint16_t m_rudderCommand;
	uint16_t m_sailCommand;
	uint8_t m_windVaneCommand;
	bool m_actuatorCommandsReady;
	bool m_windVaneCommandReady;
};
