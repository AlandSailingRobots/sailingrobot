#ifndef H_XBEE
#define H_XBEE
#include <string>

class xBee {
private:
	std::string m_receivedBuffer;
public:


	/*
		initializes the xBee with given baudrate 
	*/
	int init(int baudRate);

	/*
		Reads incoming echoes for the specified device
	*/
	std::string readOutput(int fd);


	/*
		Sends a string through specified device
	*/
	void printInput(std::string input, int fd);


	/*
		Sends a string to output with a given id
	*/
	void sendXML(int fd, std::string);





};

#endif
