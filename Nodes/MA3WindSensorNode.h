/****************************************************************************************
 *
 * File:
 * 		MA3WindSensorNode.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include "ActiveNode.h"


class MA3WindSensorNode : public ActiveNode {
public:
	MA3WindSensorNode(MessageBus& msgbus, int channel);
	virtual ~MA3WindSensorNode();

	///----------------------------------------------------------------------------------
	/// Checks to see if it can get a value from the channel
	///----------------------------------------------------------------------------------
	virtual bool init();

	///----------------------------------------------------------------------------------
	/// Currently doesn't process any messages.
	///----------------------------------------------------------------------------------
	virtual void processMessage(const Message* message) { }

	///----------------------------------------------------------------------------------
	/// Starts the main thread that will check up on the sensor
	///----------------------------------------------------------------------------------
	virtual void start();

	///----------------------------------------------------------------------------------
	/// Returns the wind direction
	///----------------------------------------------------------------------------------
	int readWindDirection();

	///----------------------------------------------------------------------------------
	/// Reads the sensor's value and posts a message to the message bus
	///----------------------------------------------------------------------------------
	static void ma3ThreadFunc(void* nodePtr);
private:
	int 		m_channel;
	bool 		m_running;
};
