/****************************************************************************************
 *
 * File:
 * 		ArduinoNode.h
 *
 * Purpose:
 *		The Arduino node communicates with a _______
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"
#include "i2ccontroller/I2CController.h"

#define DEFAULT_I2C_ADDRESS_PRESSURE 0x07

#define STARTBYTE 0x0F


class ArduinoNode : public ActiveNode {
public:
	ArduinoNode(MessageBus& msgBus);

	///----------------------------------------------------------------------------------
	/// Attempts to connect to the Arduino.
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// This function should be used to start the active nodes thread.
 	///----------------------------------------------------------------------------------
	void start();

	void processMessage(const Message* msg);


private:
	static void ArduinoThreadFunc(void* nodePtr);

	I2CController 	m_I2C;
	bool 			m_Initialised;
    int             m_pressure;
    int             m_rudder;
    int             m_sheet;
    int             m_battery;
};
