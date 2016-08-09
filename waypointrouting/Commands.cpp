#include "Commands.h"
#include <math.h>
#include "utility/Utility.h"
#include <iostream>

Commands::Commands() :
	m_starboardExtreme(1),
	m_portExtreme(-1),
	m_closeReach(0.1),
	m_running(1)
{}

Commands::~Commands(){}


double Commands::rudderCommand(double courseToSteer, double heading,double maxCommandAngle) {
	double offCourse = Utility::limitAngleRange(courseToSteer - heading);

	if (offCourse > 180) {
		offCourse -= 360;
	}

	if (offCourse > maxCommandAngle*-1 && offCourse <= maxCommandAngle) {
		return m_portExtreme + (offCourse +maxCommandAngle ) * (m_starboardExtreme - m_portExtreme) / (maxCommandAngle*2.0);
	} else {

		if (offCourse > maxCommandAngle) {
			offCourse = m_starboardExtreme;
		}
		else {
			offCourse = m_portExtreme;
		}

	}

	return offCourse;
}


double Commands::sailCommand(double relativeWindDirection) {
	relativeWindDirection = Utility::limitAngleRange(relativeWindDirection);

	if (relativeWindDirection > 180)
	{
		relativeWindDirection = 360 - relativeWindDirection;
	}
	if (relativeWindDirection < 45)
	{
		return m_closeReach;
	}

	return m_closeReach + (relativeWindDirection - 45.0) * (m_running - m_closeReach) / 135.0;
}


double Commands::runningSailCommand() {
	return m_running;
}
