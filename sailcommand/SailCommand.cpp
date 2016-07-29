#include "SailCommand.h"


SailCommand::SailCommand() {

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