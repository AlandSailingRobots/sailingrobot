#ifndef __COMMANDS_H__
#define __COMMANDS_H__

class Commands
{
public:
	Commands();
	~Commands();

	double rudderCommand(double courseToSteer, double heading,double maxCommandAngle);
	double sailCommand(double relativeWindDirection);
	double runningSailCommand();

private:
	const double m_starboardExtreme;
	const double m_portExtreme;
	const double m_closeReach;
	const double m_running;
};

#endif