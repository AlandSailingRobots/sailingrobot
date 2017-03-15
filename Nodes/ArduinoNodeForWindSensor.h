/****************************************************************************************
 *
 * File:
 * 		ArduinoNodeForWindSensor.h
 *
 * Purpose:
 *		The Arduino node communicates with the arduino. Sends data about the wind direction.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"
#include "i2ccontroller/I2CController.h"



class ArduinoNodeForWindSensor : public ActiveNode {
public:
	ArduinoNodeForWindSensor(MessageBus& msgBus);

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
    float             m_windDirection;
};