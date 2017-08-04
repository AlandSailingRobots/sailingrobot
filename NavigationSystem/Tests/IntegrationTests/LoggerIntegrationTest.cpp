#include "logger/Logger.h"
#include <iostream>


int main(int argc, char** argv) {
	Logger log;
	try {
		if (log.init("testLogger")) {
			std::cout<< "successfull init"<<std::endl;
		}
		else {
			std::cout<< "error in init"<<std::endl;
			return 0;
		}
	}
	catch (const char* e) {
		std::cout<< "exeption thrown: "<< e <<std::endl;
		return 0;
	}
	log.info("test message");
	log.info("an other one");
	log.error("FUUU");
}
