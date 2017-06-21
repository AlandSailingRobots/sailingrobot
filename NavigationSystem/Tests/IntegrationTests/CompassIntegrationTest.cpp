#include <iostream>
#include <unistd.h>
#include "Compass/HMC6343.h"
#include "Compass/Compass.h"
#include "Compass/MockCompass.h"

int main() {
	std::cout << "Compass example" << std::endl;
	std::cout << "---------------" << std::endl;
    std::cout << "creating object" << std::endl;
    unsigned int headingBufferSize = 1;
    HMC6343 c(headingBufferSize);
//    MockCompass c;
	Compass* compass = &c;
	std::cout << "object created" << std::endl;

	std::cout << "initialization: " << compass->init() << std::endl;

	while(true) {
		compass->readValues();
                compass->readAccel();
		std::cout << "heading : " << compass->getHeading() << "\t"
				<< "pitch : " << compass->getPitch() << "\t"
				<< "roll : " << compass->getRoll() << "\t"
				<< "accelX: "<< compass->getAccelX()<< "\t"
                                << "accelY: "<< compass->getAccelY()<< "\t"
                                << "accelZ: "<< compass->getAccelZ()<<std::endl;
		usleep(2000);
	}
}
 
