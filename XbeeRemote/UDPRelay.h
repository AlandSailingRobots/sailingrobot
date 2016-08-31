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
#include <stdint.h>


#ifdef __linux__

typedef int UDPSocket;

#elif _WIN32

#include <winsock2.h>

typedef SOCKET UDPSocket;

#endif


class UDPRelay {
public:
	UDPRelay(std::vector<int> ports, std::string address);
	virtual ~UDPRelay();

	///----------------------------------------------------------------------------------
	/// Writes a UDP packet to the address and list of ports.
	///----------------------------------------------------------------------------------
	void write(const char *s,...);

	///----------------------------------------------------------------------------------
	/// Writes a UDP packet to the address and list of ports.
	///----------------------------------------------------------------------------------
	void write(uint8_t* data, uint16_t size);

	///----------------------------------------------------------------------------------
	/// Receive a UDP packet
	///----------------------------------------------------------------------------------
	//std::string receive();

	bool isSetup() { return m_setup; }
private:
	///----------------------------------------------------------------------------------
	/// Sends a UDP packet
	///----------------------------------------------------------------------------------
	void send(const char* msg);

	void send(uint8_t* data, uint16_t size);

	std::string 		m_destAddress;
	std::vector<int> 	m_ports;
	UDPSocket			m_socket;
	bool				m_setup;
};
