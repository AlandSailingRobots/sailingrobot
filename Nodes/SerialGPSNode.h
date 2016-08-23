/****************************************************************************************
 *
 * File:
 * 		SerialGPSNode.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include "ActiveNode.h"


class SerialGPSNode : public ActiveNode {
public:
	///----------------------------------------------------------------------------------
	/// Default constructor
	///----------------------------------------------------------------------------------
	SerialGPSNode(MessageBus& msgBus);

	///----------------------------------------------------------------------------------
	/// Default deconstructor
	///----------------------------------------------------------------------------------
	virtual ~SerialGPSNode();

	///----------------------------------------------------------------------------------
	/// Initialises the serial port the GPS is on.
	///----------------------------------------------------------------------------------
	virtual bool init();

	///----------------------------------------------------------------------------------
	/// Called by the message bus when it has a message for this node.
	///----------------------------------------------------------------------------------
	virtual void processMessage(const Message* message);

	///----------------------------------------------------------------------------------
	/// Starts the node's thread.
	///----------------------------------------------------------------------------------
	virtual void start();

	///----------------------------------------------------------------------------------
	/// Reads the serial line and attempts to return a complete NMEA0183 line. This
	/// function will attempt to read a line several times before returning if it is
	/// unable to do so the first time. This wait is around 25 Milliseconds.
	///
	/// @param nmeaBuffer			Pointer to a character buffer to fill with NMEA data.
	/// @param maxLength			The max number of characters the nmeaBuffer can hold.
	/// @returns					The number of characters read.
	///----------------------------------------------------------------------------------
	uint8_t readNMEALine(char* nmeaBuffer, uint8_t maxLength);

	///-----------------------------------------------------------------------------------
	/// Processes an NMEA string and attempts to extract data from it. That data is then
	/// stored in the node's data cache and can be retrieved using the getter functions
	///
	/// @param nmeaBUffer			A pointer to the null terminator buffer
	///-----------------------------------------------------------------------------------
	void processNMEA(char* nmeaBuffer);

	///-----------------------------------------------------------------------------------
	/// Getters
	///-----------------------------------------------------------------------------------

	bool 			hasFix() 		{ return m_HasFix; }
	float 			longitude() 	{ return m_Lon; }
	float			latitude()		{ return m_Lat; }
	float			speed()			{ return m_Speed; }
	float			Course()		{ return m_Course; }
	unsigned long 	unixTime() 		{ return m_UnixTime; }


private:
	///----------------------------------------------------------------------------------
	/// Starts the Serial GPS's thread that will actively put GPS data messages onto the
	/// message queue.
	///----------------------------------------------------------------------------------
	static void GPSThreadFunc(void* nodePtr);

	int 		m_FD;
	uint16_t	m_BaudRate;
	std::string m_PortName;

	// Cache
	bool 			m_HasFix;
	float 			m_Lon;
	float 			m_Lat;
	float 			m_Speed;
	float 			m_Course;
	unsigned long 	m_UnixTime;

	bool 			m_keepRunning;
};
