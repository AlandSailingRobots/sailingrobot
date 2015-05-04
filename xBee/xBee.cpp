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


	// this setting needs a udev rule in host system to work (alternative is dynamic USB-slot)
	// see: http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/
	std::string deviceName = "/dev/xbee";







	if((m_fd = serialOpen(deviceName.c_str(), baudRate)) < 0) {
		throw "CV7::openPort: Unable to connect";
	}


	

	return m_fd;
	





}