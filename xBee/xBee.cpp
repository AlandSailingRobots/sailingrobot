#include "xBee.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <fstream>
#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>


std::string xBee::readOutput(int fd){

	std::string printer;
	std::string result;

	int available = serialDataAvail(fd);

	if (available != -1){
		while (available > 0){
			printer += serialGetchar(fd);
			available--;
		}
		m_receivedBuffer += printer;
	}else{
		printer = "Data reception failed.";
	}



	std::size_t pos_start = m_receivedBuffer.find("<message>");
	std::size_t pos_end = m_receivedBuffer.find("</message>");

	if (pos_start != std::string::npos &&
		pos_end != std::string::npos &&
		pos_start < pos_end) {
    	std::cout << "XML message start and end tags found!" << '\n' << 
    			     "Start tag pos: " << pos_start << '\n' << "End tag pos: " << pos_end << std::endl;

		result = m_receivedBuffer.substr(pos_start, pos_end + pos_start); 
		std::cout << "Resulting XML: " << result << std::endl;

		m_receivedBuffer.clear();
	}

	if(m_receivedBuffer.size() > 500) {
		m_receivedBuffer.clear();
	}

	return result;
}

void xBee::printInput(std::string input, int fd){

	int loops = 1;

	while (loops > 0){

		serialPuts(fd, input.c_str());
		loops--;

	}
}

void xBee::sendXML(int fd, std::string output){

	serialPuts(fd, output.c_str());
}


int xBee::init(int baudRate){

	// this setting needs a udev rule in host system to work (alternative is dynamic USB-slot)
	// see Static_USB_Device_Names in project installation folder
	std::string deviceName = "/dev/xbee";

	int fd;


	if((fd = serialOpen(deviceName.c_str(), baudRate)) < 0) {
		throw "xBee::init: Unable to connect";
	}
	
	return fd;
	
}
