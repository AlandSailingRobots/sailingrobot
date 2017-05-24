#include "SoftSailControl.h"

#define NORM_SAIL_COMMAND 0.6958


SoftSailControl::SoftSailControl() {

}

SoftSailControl::~SoftSailControl() {

}

int SoftSailControl::getCommand(double command) {
	//0=closereach, 1=running
	if (command < 0)
		command = 0;
	if (command > 1)
		command = 1;

	return (m_runningCommand - m_closeReachCommand) * command + m_closeReachCommand;
}

void SoftSailControl::setCommandValues(int closeReach, int running) {
	m_closeReachCommand = closeReach;
	m_runningCommand = running;
}


int SoftSailControl::getSailCommand(double apparentWindDirection) {
  double sailCommand = fabs(((MIN_SAIL_ANGLE - MAX_SAIL_ANGLE) / M_PI) * fabs(apparentWindDirection + MAX_SAIL_ANGLE));/*!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)*/

	if (cos(apparentWindDirection + M_PI) + cos(MAX_SAIL_ANGLE) <0 )
	{
		sailCommand = MIN_SAIL_ANGLE;
	}

  return getCommand(sailCommand/NORM_SAIL_COMMAND);
}
