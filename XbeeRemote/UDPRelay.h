/****************************************************************************************
 *
 * File:
 * 		UDPRelay.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once


#include <vector>
#include <strings.h>
#include <string>


#ifdef __linux__

typedef int UDPSocket;

#elif _WIN32

#include <winsock2.h>

typedef SOCKET UDPSocket;

#endif


class UDPRelay {
public:
	UDPRelay(std::vector<int> ports, std::string address, int receivePort);
	virtual ~UDPRelay();

	///----------------------------------------------------------------------------------
	/// Writes a UDP packet to the address and list of ports.
	///----------------------------------------------------------------------------------
	void write(const char *s,...);

	///----------------------------------------------------------------------------------
	/// Receive a UDP packet
	///----------------------------------------------------------------------------------
	//std::string receive();

private:
	///----------------------------------------------------------------------------------
	/// Sends a UDP packet
	///----------------------------------------------------------------------------------
	void send(const char* msg);

	std::string 		m_destAddress;
	std::vector<int> 	m_ports;
	UDPSocket			m_socket;
	UDPSocket			m_socket;
};
