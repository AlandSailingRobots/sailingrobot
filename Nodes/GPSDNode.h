/****************************************************************************************
 *
 * File:
 * 		GPSDNode.h
 *
 * Purpose:
 *		A GPSD node that uses the GPSD library to provide GPS data to the message bus.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"
#include <libgpsmm.h>


class GPSDNode : public ActiveNode {
public:
	GPSDNode(MessageBus& msgBus);
	
	~GPSDNode() { }

	///----------------------------------------------------------------------------------
	/// Initialises a connection gpsd.
	///
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
	/// Processes DataRequest messages.
	///
	///----------------------------------------------------------------------------------
	void processMessage(Message* msgPtr);

	///----------------------------------------------------------------------------------
 	/// This function starts the GPS thread
 	///
 	///----------------------------------------------------------------------------------
	void start();
private:
	static void GPSThreadFunc(void* nodePtr);

	bool 	m_Initialised;
	gpsmm* 	m_GpsConnection;
};
