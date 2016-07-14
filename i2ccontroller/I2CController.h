#ifndef __I2CCONTROLLER_H__
#define __I2CCONTROLLER_H__

#include <vector>
#include <stdint.h>
#include <wiringPiI2C.h>
#include <wiringPi.h> // delay
#include <string>
#include <mutex>
#include "AnalogArduino/myWiringI2C.h"

class I2CController {

	public:
		I2CController(int fd);
		~I2CController();

		int setup(int address);
		std::vector<uint8_t> readGeneric(uint8_t command);
		int write(int data);
		int read();
		int readBlock();

		void beginTransmission();
		void endTransmission();

	private:
		static std::mutex m_mutex;
		bool locked;
		int m_fd;
};

#endif
