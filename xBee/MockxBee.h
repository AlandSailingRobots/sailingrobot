#ifndef H_MOCKXBEE
#define H_MOCKXBEE
#include <string>

class MockxBee {
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
