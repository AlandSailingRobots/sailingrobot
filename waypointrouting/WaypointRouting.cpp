#include "WaypointRouting.h"
#include <math.h>
#include "coursecalculation/CourseCalculation.h"
#include "utility/Timer.h"
#include "utility/Utility.h"
#include "models/WaypointModel.h"
#include "models/SystemStateModel.h"


WaypointRouting::WaypointRouting(WaypointModel waypoint, double innerRadiusRatio,
		double tackAngle, double maxTackAngle, double minTackSpeed, double sectorAngle, 
		 double maxCommandAngle, double  rudderSpeedMin) :
	m_tackAngleHandler(tackAngle, maxTackAngle, minTackSpeed),
	m_commandHandler(),
	m_waypoint(waypoint),
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

void WaypointRouting::getCommands(double & rudder, double & sail, PositionModel boat,
	double trueWindDirection, double heading, SystemStateModel systemStateModel) {

	double speed = Utility::directionAdjustedSpeed(systemStateModel.gpsModel.heading,
												  										systemStateModel.compassModel.heading,
												  										systemStateModel.gpsModel.speed);
	double commandAngle = m_maxCommandAngle;
	
	// If we have reached the waypoint but want to stay there for a bit we want to sail into the
	// wind so we "hover" at the waypoint
	if (m_waypoint.time > 0 && reachedRadius(m_waypoint.radius * m_innerRadiusRatio, boat)) {
		
		if(speed > m_rudderSpeedMin) {
			commandAngle = speed / m_rudderSpeedMin * m_maxCommandAngle;
		}
		
		m_courseToSteer = trueWindDirection;
		rudder = m_commandHandler.rudderCommand(m_courseToSteer, heading,commandAngle);
		sail = m_commandHandler.runningSailCommand();
	} 
	//  Normal sailing towards a waypoint
	else {

		m_courseCalc.setTackAngle(m_tackAngleHandler.adjustedTackAngle(systemStateModel));

		if(speed > m_rudderSpeedMin) {
			commandAngle = speed/m_rudderSpeedMin * m_maxCommandAngle;		
		}
		
		m_courseToSteer = m_courseCalc.calculateCourseToSteer(boat, m_waypoint, trueWindDirection);
		rudder = m_commandHandler.rudderCommand(m_courseToSteer, heading,commandAngle);
		sail = m_commandHandler.sailCommand(systemStateModel.windsensorModel.direction);
	}

	if(!adjustSteering(systemStateModel.windsensorModel.direction)) {
		m_lastRWD = systemStateModel.windsensorModel.direction;
		sail = m_lastSail;
		return;
	}

	m_lastRWD = systemStateModel.windsensorModel.direction;
	m_lastSail = sail;
}


bool WaypointRouting::nextWaypoint(PositionModel boat)
{
	bool nextWaypoint = false;
	if (m_waypoint.time > 0)
	{
		if (m_waypointTimer.timeReached(m_waypoint.time))
			nextWaypoint = true;
		if (reachedRadius(m_waypoint.radius, boat))
			m_waypointTimer.start();
	}
	else
	{
		if (reachedRadius(m_waypoint.radius, boat))
			nextWaypoint = true;
	}
	return nextWaypoint;
}

void WaypointRouting::setWaypoint(WaypointModel waypoint)
{
	m_waypoint = waypoint;
	m_waypointTimer.stop();
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

bool WaypointRouting::reachedRadius(double radius, PositionModel boat) const
{
	bool reachedRadius = false;
	double dtw = m_courseMath.calculateDTW(boat, m_waypoint.positionModel);
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
