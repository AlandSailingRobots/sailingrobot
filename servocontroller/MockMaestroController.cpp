/*
 * MockMaestroController.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: sailbot
 */

#include "MockMaestroController.h"

MockMaestroController::MockMaestroController() {

}

MockMaestroController::~MockMaestroController() {

}

void MockMaestroController::setPort(std::string portName) {

}

void MockMaestroController::writeCommand(unsigned char type, int channel = -1, int value = -1) {

}

int MockMaestroController::readRespons() {
	return 1;
}

int MockMaestroController::getError() {
	return 0;
}

void MockMaestroController::openPort() {

}

bool MockMaestroController::isOpenPort() {
	return true;
}
