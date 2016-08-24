/****************************************************************************************
 *
 * File:
 * 		RazorCompassNode.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include "ActiveNode.h"


class RazorCompassNode : public ActiveNode {
public:
	RazorCompassNode(MessageBus& msgBus);

	virtual ~RazorCompassNode();

	///----------------------------------------------------------------------------------
	/// Opens up a serial port
	///
	/// @returns		Returns false if it is unable to get a serial handle
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
	/// This function should be used to start the active nodes thread.
	///----------------------------------------------------------------------------------
	void start();

	///----------------------------------------------------------------------------------
	/// Doesn't process any messages.
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Reads the serial line and attempts to return a complete data line. This
	/// function will attempt to read a line several times before returning if it is
	/// unable to do so the first time. This wait is around 25 Milliseconds.
	///
	/// @param buffer				Pointer to a character buffer to fill with data.
	/// @param maxLength			The max number of characters the nmeaBuffer can hold.
	/// @returns					The number of characters read.
	///----------------------------------------------------------------------------------
	uint8_t readLine(char* buffer, uint8_t maxLength);

	///----------------------------------------------------------------------------------
	/// Reads the heading, pitch and roll from the compass.
	///----------------------------------------------------------------------------------
	bool parseData(char* nmeaBuffer, float& heading, float& pitch, float& roll);

protected:

	///----------------------------------------------------------------------------------
	/// Reads the I2C Compass's state every x milliseconds, see COMPASS_SENSOR_SLEEP_MS
	/// for information on the timing in HMC6343Node.cpp. The state is then published to
	/// the message bus
	///----------------------------------------------------------------------------------
	static void RazorThreadFunc(void* nodePtr);

	int 			m_FD;
	bool			m_keepRunning;
};
