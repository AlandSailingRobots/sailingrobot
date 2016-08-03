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


private:
	static void ArduinoThreadFunc(void* nodePtr);
	void sendToArduino(uint32_t dataID, uint32_t CANID, uint8_t* CANData)
	void loadOutgoingMessage(const Message* message)
	void passToCANBus()
	void processIncomingCAN(uint32_t CANID, uint8_t* CANData)

	I2CController 	m_I2C;
	bool 		m_Initialised;
	int             m_pressure;
	int             m_rudder;
	int             m_sheet;
	int             m_battery;

	//adding instead of replacing for testing purposes
	uint16_t m_rudderPosition;
	uint16_t m_sailPosition;
	uint8_t m_windVaneCommand;
	bool m_actuatorPositionsReady;
	bool m_windVaneCommandReady;
};
