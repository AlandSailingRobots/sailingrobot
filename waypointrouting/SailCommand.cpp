#include "SailCommand.h"

#include <math.h>
#include <algorithm>
#include <cmath>

#define NORM_SAIL_COMMAND 0.6958

SailCommand::SailCommand() {
	  m_maxSailAngle = M_PI / 4.2f;
	  m_minSailAngle = M_PI / 32.0f;
}

SailCommand::~SailCommand() {

}

int SailCommand::getCommand(double command) {
	//0=closereach, 1=running
	if (command < 0)
		command = 0;
	if (command > 1)
		command = 1;

	return (m_runningCommand - m_closeReachCommand) * command + m_closeReachCommand;
}

void SailCommand::setCommandValues(int closeReach, int running) {
	m_closeReachCommand = closeReach;
	m_runningCommand = running;
}
