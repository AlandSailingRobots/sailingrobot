/****************************************************************************************
 *
 * File:
 * 		UDPRelay.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "UDPRelay.h"
#include <stdio.h>
#include <stdlib.h>
#include "../SystemServices/Logger.h"
#ifdef __linux__
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif
#include <time.h>
#include <cstring>


/***************************************************************************************/
/// a useful function to get a UNIX timestamp with fractional seconds.
static double gettime(){
#ifdef __linux__
    timespec ts;
	clock_gettime(CLOCK_REALTIME,&ts);

	double t = ts.tv_nsec;
	t *= 1e-9;
	t += ts.tv_sec;
	return t;
#elif _WIN32
	time_t ltime;
	time(&ltime);
	return ltime;
#endif
}


/***************************************************************************************/
UDPRelay::UDPRelay(std::vector<int> ports, std::string address)
	:m_destAddress(address), m_ports(ports)
{
#if _WIN32
	int iResult;
	WSADATA wsaData;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup failed with error: %d\n", iResult);
	}
#endif

	m_socket = socket(AF_INET,SOCK_DGRAM,0);
	if(m_socket < 0)
	{
		Logger::error("%s:%d Cannot open UDP socket", __FILE__, __LINE__);
	}
}

/***************************************************************************************/
UDPRelay::~UDPRelay()
{
#ifdef __linux__
	close(m_socket);
#elif _WIN32
	closesocket(m_socket);
#endif
}

/***************************************************************************************/
void UDPRelay::write(const char *s,...)
{
	va_list args;
	va_start(args,s);
	char buf[1024];
	sprintf(buf,"time=%f ",gettime());
	vsnprintf(buf+strlen(buf),1024-strlen(buf),s,args);
	send(buf); // SEND TO
	printf("%s\n", buf);
	va_end(args);
}

/***************************************************************************************/
void UDPRelay::send(const char* msg)
{
	for(unsigned int i = 0; i < m_ports.size(); i++)
	{
		sockaddr_in servaddr;

		memset(&servaddr, 0, sizeof(servaddr));
		servaddr.sin_family = AF_INET;
		servaddr.sin_addr.s_addr = inet_addr(m_destAddress.c_str());
		servaddr.sin_port = htons(m_ports[i]);

		char opt=1;
		setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(int));

		if (sendto(m_socket, msg, strlen(msg)+1, 0, // +1 to include terminator
				   (sockaddr*)&servaddr, sizeof(servaddr)) < 0){
			Logger::warning("%s:%d Cannot send message %s", __FILE__, __LINE__, msg);
		}
	}
}
