#ifndef __WAYPOINTROUTING_H__
#define __WAYPOINTROUTING_H__

#include "utility/CourseCalculation.h"
#include "utility/Timer.h"
#include "utility/Utility.h"
#include "TackAngle.h"
#include "Commands.h"


class WaypointRouting
{
public:
	WaypointRouting(double lon, double lat, int radius, double innerRadiusRatio,
		double tackAngle, double maxTackAngle, double minTackSpeed, double sectorAngle, 
		double maxCommandAngle, double  rudderSpeedMin);
	WaypointRouting(const WaypointRouting &) = delete;
	WaypointRouting & operator=(const WaypointRouting &) = delete;	
	~WaypointRouting();

	void getCommands(double & rudder, double & sail, double gpsLon, double gpsLat, int radius, int stayTime,
	double trueWindDirection, double heading, double gpsHeading, double gpsSpeed, double compassHeading, double windsensorDir);

	void setCourseCalcValues(double tackAngle, double sectorAngle);
	void setInnerRadiusRatio(double ratio);

	void setWaypointData(double lon, double lat, int radius);

	double getDTW();
	double getBTW();
	double getTWD();
	double getCTS();
	bool getTack();
	bool getGoingStarboard();

	void setUpdateInterval(double updateInterval);
	void setMinimumDegreeLimit(double degLimit);

	
private:

	bool reachedRadius(double radius, double gpsLon, double gpsLat) const;
	bool adjustSteering(double relativeWindDirection);

	double m_lon;
	double m_lat;
	int m_radius;
	TackAngle m_tackAngleHandler;
	Commands m_commandHandler;
	CourseCalculation m_courseCalc;
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
