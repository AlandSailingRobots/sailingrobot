#include "xBee.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <wiringSerial.h>
#include <iostream>



std::string xBee::readOutput(int fd){

	std::string printer;


	while (serialGetchar(fd) != -1){

		printer += serialGetchar(fd);
		printer += "x";




	}

	
	printer = "";

	return printer;






}


int xBee::init(){

	std::string portName = "/dev/ttyUSB0";



	if((m_fd = serialOpen(portName.c_str(), 115200)) < 0) {
		throw "CV7::openPort: Unable to connect";
	}


	

	return m_fd;
	





}