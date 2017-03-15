#include "WaypointRouting.h"
#include <math.h>
#include "SystemServices/Timer.h"
#include "Math/Utility.h"


WaypointRouting::WaypointRouting(double lon, double lat, int radius, double innerRadiusRatio,
		double tackAngle, double maxTackAngle, double minTackSpeed, double sectorAngle,
		double maxCommandAngle, double  rudderSpeedMin) :
	m_lon(lon),
	m_lat(lat),
	m_radius(radius),
	m_tackAngleHandler(tackAngle, maxTackAngle, minTackSpeed),
	m_commandHandler(),
	m_innerRadiusRatio(innerRadiusRatio),
	m_courseToSteer(0),
	m_maxCommandAngle(maxCommandAngle),
	m_rudderSpeedMin(rudderSpeedMin),
	m_updateInterval(0.0),
	m_degLimit(0.0),
	m_lastSail(0.0),
	m_timePassed(0.0)
{
	m_courseCalc.setTackAngle(tackAngle);
	m_courseCalc.setSectorAngle(sectorAngle);
	m_lastRWD = 0.0;
	m_sailControlTimer.start();
}

WaypointRouting::~WaypointRouting()
{
}

/* Uses:
	* GPS heading
	* compass heading
	* GPS speed
*/

void WaypointRouting::getCommands(double & rudder, double & sail, double gpsLon, double gpsLat, int radius, int stayTime,
	double trueWindDirection, double heading, double gpsHeading, double gpsSpeed, double compassHeading, double windsensorDir) {

	double speed = Utility::directionAdjustedSpeed(gpsHeading,compassHeading, gpsSpeed);
	double commandAngle = m_maxCommandAngle;
	// m_courseCalc.setTackAngle(m_tackAngleHandler.adjustedTackAngle(gpsHeading,compassHeading, gpsSpeed));

	// if(speed > m_rudderSpeedMin) {
	// 	commandAngle = speed/m_rudderSpeedMin * m_maxCommandAngle;
	// }

	// m_courseToSteer = m_courseCalc.calculateCourseToSteer(gpsLon, gpsLat, m_lon, m_lat, radius, trueWindDirection);
	// rudder = m_commandHandler.rudderCommand(m_courseToSteer, heading,commandAngle);
	// sail = m_commandHandler.sailCommand(windsensorDir);

	if (stayTime > 0 && reachedRadius(radius * m_innerRadiusRatio, gpsLon, gpsLat)) 
	{	
		if(speed > m_rudderSpeedMin) {
			commandAngle = speed/m_rudderSpeedMin * m_maxCommandAngle;		
		}

		m_courseToSteer = trueWindDirection;
		rudder = m_commandHandler.rudderCommand(m_courseToSteer, heading,commandAngle);
		sail = m_commandHandler.runningSailCommand();
	} 
	else {
		m_courseCalc.setTackAngle(m_tackAngleHandler.adjustedTackAngle(gpsHeading,compassHeading, gpsSpeed));

		if(speed > m_rudderSpeedMin) {
			commandAngle = speed/m_rudderSpeedMin * m_maxCommandAngle;
		}

		m_courseToSteer = m_courseCalc.calculateCourseToSteer(gpsLon, gpsLat, m_lon, m_lat, radius, trueWindDirection);
		rudder = m_commandHandler.rudderCommand(m_courseToSteer, heading,commandAngle);
		sail = m_commandHandler.sailCommand(windsensorDir);
	}

	if(!adjustSteering(windsensorDir)) {
		m_lastRWD = windsensorDir;
		sail = m_lastSail;
		return;
	}

	m_lastRWD = windsensorDir;
	m_lastSail = sail;
}


void WaypointRouting::setCourseCalcValues(double tackAngle, double sectorAngle)
{
	m_courseCalc.setTackAngle(tackAngle);
	m_courseCalc.setSectorAngle(sectorAngle);
}

void WaypointRouting::setInnerRadiusRatio(double ratio)
{
	m_innerRadiusRatio = ratio;
}

void WaypointRouting::setWaypointData(double lon, double lat, int radius)
{
	m_lon = lon;
	m_lat = lat;
	m_radius = radius;
}

double WaypointRouting::getDTW()
{
	return m_courseCalc.getDTW();
}

double WaypointRouting::getBTW()
{
	return m_courseCalc.getBTW();
}

double WaypointRouting::getTWD()
{
	return m_courseCalc.getTWD();
}

double WaypointRouting::getCTS()
{
	return m_courseToSteer;
}

bool WaypointRouting::getTack()
{
	return m_courseCalc.getTack();
}

bool WaypointRouting::getGoingStarboard()
{
	return m_courseCalc.getGoingStarboard();
}

void WaypointRouting::setUpdateInterval(double updateInterval) {
	m_updateInterval = updateInterval;
}

void WaypointRouting::setMinimumDegreeLimit(double degLimit) {
	m_degLimit = degLimit;
}

bool WaypointRouting::reachedRadius(double radius, double gpsLon, double gpsLat) const
{
	bool reachedRadius = false;
	double dtw = CourseMath::calculateDTW(gpsLon, gpsLat, m_lon, m_lat);
	if (dtw < radius)
		reachedRadius = true;
	return reachedRadius;
}

bool WaypointRouting::adjustSteering(double relativeWindDirection) {
	m_timePassed += m_sailControlTimer.timePassed();

	if(m_timePassed > m_updateInterval) {
		m_timePassed = 0;

		if(Utility::angleDifference(relativeWindDirection, m_lastRWD) > m_degLimit) {
			m_sailControlTimer.reset();
			return true;
		}
	}

	return false;
}
