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

	int available = serialDataAvail(fd);

	if (available != -1){
		while (available > 0){
			printer += serialGetchar(fd);
			available--;
		}
	}else{
		printer = "Data reception failed.";
	}


	if (printer.find("<message>") != std::string::npos &&
		printer.find("</message>") != std::string::npos) {
    	std::cout << "XML message start and end tags found!" << '\n';
	}

	return printer;
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

	// Måste få in ett handtag till databas hanteraren som nu bara finns i SailingRobot klassen.

	//	Hämtar ett heltal (1 eller 0) som visar om xbeen skall skicka och ta emot data.
	//	m_dbHandler.retriveCellAsInt("configs", "1", "xb_send")
	//	m_dbHandler.retriveCellAsInt("configs", "1", "xb_recv")

	// this setting needs a udev rule in host system to work (alternative is dynamic USB-slot)
	// see Static_USB_Device_Names in project installation folder
	std::string deviceName = "/dev/xbee";

	int fd;


	if((fd = serialOpen(deviceName.c_str(), baudRate)) < 0) {
		throw "xBee::init: Unable to connect";
	}
	
	return fd;
	
}
