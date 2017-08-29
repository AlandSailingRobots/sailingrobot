/****************************************************************************************
 *
 * File:
 * 		LinuxSerialDataLink.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include "DataLink.h"


class LinuxSerialDataLink : public DataLink {
public:
	LinuxSerialDataLink(std::string port, uint16_t baudRate);

	virtual ~LinuxSerialDataLink();

	///----------------------------------------------------------------------------------
	/// Initialises the data link. This function should ensure that the hardware layer
	/// is up and functioning.
	///----------------------------------------------------------------------------------
	virtual bool initialise(uint16_t frameSize);

	///----------------------------------------------------------------------------------
	/// Sends an AT command over the data link.
	///----------------------------------------------------------------------------------
	virtual std::string sendATCommand(std::string command, uint16_t responseSize);

protected:
	virtual void writeData(const uint8_t* data, uint8_t size);
	virtual void readData(uint8_t* data, uint8_t size);
	virtual int readByte();
	virtual void writeByte(uint8_t byte);
	virtual bool dataAvailable();

private:
	int 					m_handle; // Xbee fd
	std::string				m_port;
	uint16_t				m_baudRate;
};
