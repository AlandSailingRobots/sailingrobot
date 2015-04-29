#ifndef H_XBEE
#define H_XBEE
#include <string>

class xBee {
public:


	/*
		initializes the xBee with given number of usbport and baudrate
	*/
	int init(int usbPort, int baudRate);

	/*
		Reads incoming echoes for the specified device
	*/
	std::string readOutput(int fd);


	/*
		Sends a string through specified device
	*/
	void printInput(std::string input, int fd);


	/*
		Sends sourcefile.xml as string through specified device
	*/
	void sendXML(int fd);

	int m_fd = -1;




};

#endif
