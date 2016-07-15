/****************************************************************************************
 *
 * File:
 * 		HMC6343Node.h
 *
 * Purpose:
 *		The HMC6343 node communicates with a HMC6343 compass and provides CompassDataMsgs
 *		to the message bus.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"
#include "i2ccontroller/I2CController.h"


class HMC6343Node : public ActiveNode {
public:
	HMC6343Node(MessageBus& msgBus, const int headingBufferSize = 10);

	///----------------------------------------------------------------------------------
	/// Attempts to connect to the HMC6343 compass.
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// This function should be used to start the active nodes thread.
 	///----------------------------------------------------------------------------------
	void start();

	void processMessage(const Message* msg);

private:
	///----------------------------------------------------------------------------------
	 /// Reads the heading, pitch and roll from the compass.
	 ///----------------------------------------------------------------------------------
	bool readData(float& heading, float& pitch, float& roll);

	static void HMC6343ThreadFunc(void* nodePtr);

	I2CController 	m_I2C;
	bool 			m_Initialised;
	const int		m_HeadingBufferSize;
};
