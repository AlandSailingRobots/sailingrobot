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

// Magic numbers correspond to the compass commands, see the datasheet for more info.
enum class CompassOrientation {
	COM_ORIENT_LEVEL = 0x72,
	COM_ORIENT_SIDEWAYS = 0x73,
	COM_ORIENT_FLATFRONT = 0x74
};


class HMC6343Node : public ActiveNode {
public:
	HMC6343Node(MessageBus& msgBus, const int headingBufferSize = 10);

	virtual ~HMC6343Node() { }

	///----------------------------------------------------------------------------------
	/// Attempts to connect to the HMC6343 compass.
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// This function should be used to start the active nodes thread.
 	///----------------------------------------------------------------------------------
	void start();

	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Sets the compass's orientation.
	///----------------------------------------------------------------------------------
	bool setOrientation(CompassOrientation orientation);

	///----------------------------------------------------------------------------------
	/// Reads the heading, pitch and roll from the compass.
	///----------------------------------------------------------------------------------
	bool readData(float& heading, float& pitch, float& roll);

protected:
	static void HMC6343ThreadFunc(void* nodePtr);

	I2CController 	m_I2C;
	bool 			m_Initialised;
	const int		m_HeadingBufferSize;
};
