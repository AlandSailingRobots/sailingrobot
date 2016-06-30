#ifndef __WAYPOINTROUTING_H__
#define __WAYPOINTROUTING_H__

#include "coursecalculation/CourseCalculation.h"
#include "utility/Timer.h"
#include "utility/Utility.h"
#include "TackAngle.h"
#include "Commands.h"
#include "models/WaypointModel.h"

class PositionModel;
class SystemStateModel;

class WaypointRouting
{
public:
	WaypointRouting(WaypointModel waypoint, double innerRadiusRatio,
		double tackAngle, double maxTackAngle, double minTackSpeed, double sectorAngle, 
		double maxCommandAngle, double  rudderSpeedMin);
	WaypointRouting(const WaypointRouting &) = delete;
	WaypointRouting & operator=(const WaypointRouting &) = delete;	
	~WaypointRouting();

	void getCommands(double & rudder, double & sail, PositionModel boat,
		double trueWindDirection, double heading, SystemStateModel systemStateModel);
	bool nextWaypoint(PositionModel boat);
	void setWaypoint(WaypointModel waypoint);
	void setCourseCalcValues(double tackAngle, double sectorAngle);
	void setInnerRadiusRatio(double ratio);
	
	double getDTW();
	double getBTW();
	double getTWD();
	double getCTS();
	bool getTack();
	bool getGoingStarboard();

	void setUpdateInterval(double updateInterval);
	void setMinimumDegreeLimit(double degLimit);
	
private:
	bool reachedRadius(double radius, PositionModel boat) const;

	bool adjustSteering(double relativeWindDirection);

	TackAngle m_tackAngleHandler;
	Commands m_commandHandler;
	WaypointModel m_waypoint;
	CourseCalculation m_courseCalc;
	CourseMath m_courseMath;
	double m_innerRadiusRatio;
	Timer m_waypointTimer;
	double m_courseToSteer;
	double m_maxCommandAngle;
	double m_rudderSpeedMin;

	Timer m_sailControlTimer;
	double m_updateInterval, m_degLimit;
	double m_lastSail, m_lastRWD;
	double m_timePassed;
};

#endif