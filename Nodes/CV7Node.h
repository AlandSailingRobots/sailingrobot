/****************************************************************************************
 *
 * File:
 * 		CV7Node.h
 *
 * Purpose:
 *		The CV7 node provides wind data to the system using the CV7 wind sensor.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "ActiveNode.h"
#include <map>


class CV7Node : public ActiveNode{
public:
	CV7Node(MessageBus& msgBus, std::string m_PortName, unsigned int baudRate);

	virtual ~CV7Node() { }

	///----------------------------------------------------------------------------------
 	/// Attempts to connect to the CV7 wind sensor.
 	///
 	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// Starts the wind sensors thread so that it actively pumps data into the message 
 	/// bus.
 	///
 	///----------------------------------------------------------------------------------
	void start();

	///----------------------------------------------------------------------------------
 	/// The CV7 Node only processes data request messages.
 	///
 	///----------------------------------------------------------------------------------
	void processMessage(const Message* message);

	///----------------------------------------------------------------------------------
	/// The CV7 Node's thread function.
	///
	///----------------------------------------------------------------------------------
	bool parseString(const char* buffer, float& windDir, float& windSpeed, float& windTemp) const;

private:
	///----------------------------------------------------------------------------------
 	/// The CV7 Node's thread function.
 	///
 	///----------------------------------------------------------------------------------
	static void WindSensorThread(void* nodePtr);

	bool m_Initialised;		// Indicates that the node was correctly initialised
	int m_fd;
	std::string m_PortName;
	unsigned int m_BaudRate;
	float m_MeanWindDir;
	float m_MeanWindSpeed;
	float m_MeanWindTemp;
};
