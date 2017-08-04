/****************************************************************************************
 *
 * File:
 * 		UDPReceiver.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once


#include "UDPRelay.h"


class UDPReceiver {
public:
	UDPReceiver();
	virtual ~UDPReceiver();

	bool initialise(int receivePort);

	///----------------------------------------------------------------------------------
	/// Receive a UDP packet
	///----------------------------------------------------------------------------------
	uint8_t* receive(uint16_t& size);
private:
	UDPSocket			m_socket;
};
