#ifndef H_XBEE
#define H_XBEE
#include <string>

class xBee {
public:

	int init();
	std::string readOutput(int fd);
	void printInput(std::string input, int fd);
	void sendXML(int fd);

	int m_fd = -1;




};

#endif
