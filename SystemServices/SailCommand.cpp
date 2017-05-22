#include"SailCommand.h"

#define NORM_SAIL_COMMAND 0.6958


SailCommand::SailCommand() {

}

SailCommand::~SailCommand() {

}

int SailCommand::getSailCommand(double apparentWindDirection) {
  double sailCommand = fabs(((MIN_SAIL_ANGLE - MAX_SAIL_ANGLE) / M_PI) * fabs(apparentWindDirection + MAX_SAIL_ANGLE));/*!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)*/

	if (cos(apparentWindDirection + M_PI) + cos(MAX_SAIL_ANGLE) <0 )
	{
		sailCommand = MIN_SAIL_ANGLE;
	}
	return sailCommand/NORM_SAIL_COMMAND;
}
