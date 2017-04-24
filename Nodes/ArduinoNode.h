/****************************************************************************************
 *
 * File:
 * 		ArduinoNode.h
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
#include "HardwareServices/i2ccontroller/I2CController.h"



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
	static void ArduinoThreadFunc(ActiveNode* nodePtr);

	I2CController 	m_I2C;
	bool 			m_Initialised;
    int             m_pressure;
    int             m_rudder;
    int             m_sheet;
    int             m_battery;
	int				m_RC;
};
