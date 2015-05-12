#ifndef C_MOCKXBEE
#define C_MOCKXBEE
#include "MockxBee.h"
#include <string>




std::string MockxBee::readOutput(int fd){
	return "";
}

void MockxBee::printInput(std::string input, int fd){

}

void MockxBee::sendXML(int fd, std::string output){

}

int MockxBee::init(int baudRate){
	return 1;
}

#endif
