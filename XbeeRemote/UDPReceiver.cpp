/****************************************************************************************
 *
 * File:
 * 		UDPReceiver.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "UDPReceiver.h"

#ifdef __linux__
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#elif _WIN32
#include <winsock2.h>
typedef int socklen_t;
#endif
#include <cstring>


UDPReceiver::UDPReceiver() {
	// TODO Auto-generated constructor stub

}

UDPReceiver::~UDPReceiver() {
	// TODO Auto-generated destructor stub
}

bool UDPReceiver::initialise(int receivePort)
{
	struct sockaddr_in myaddr;

	/* create a UDP socket */
	if ((m_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return false;
	}

// Set to nonblocking
#ifdef __linux__
	fcntl(m_socket, F_SETFL, O_NONBLOCK);
#elif _WIN32
	u_long iMode = 1;
	iResult = ioctlsocket(m_socket, FIONBIO, &iMode);
	if (iResult != NO_ERROR)
	{
		printf("ioctlsocket failed with error: %ld\n", iResult);
	}
#endif

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(receivePort);

	if (bind(m_socket, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		perror("bind failed");
		return false;
	}

	return true;
}

uint8_t* UDPReceiver::receive(uint16_t& size)
{
	char buffer[200];
	struct sockaddr_in remaddr;
	socklen_t addrlen = sizeof(remaddr);

	int tmpSize = recvfrom(m_socket, buffer, 200, 0, (struct sockaddr *)&remaddr, &addrlen);
	if (tmpSize > 0)
	{
		size = tmpSize;
		printf("received %d bytes\n", size);
		printf("received message: \"%s\"\n", buffer);
		uint8_t* data = new uint8_t[size];

		memcpy(data, buffer, size);
		return data;
	}

	return NULL;
}


