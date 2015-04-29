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

void xBee::sendXML(int fd){

	std::string stringfile, tmp;

	std::ifstream input("../log_output.xml");

	while(!input.eof()) {

    getline(input, tmp);
    stringfile += tmp;
	}

	int loops = 6;

	while (loops > 0){

	serialPuts(fd, stringfile.c_str());
	loops--;
	usleep(100);

	}




}


int xBee::init(int usbPort, int baudRate){

	std::string deviceName = "/dev/ttyUSB";
	deviceName += std::to_string(usbPort);







	if((m_fd = serialOpen(deviceName.c_str(), baudRate)) < 0) {
		throw "CV7::openPort: Unable to connect";
	}


	

	return m_fd;
	





}