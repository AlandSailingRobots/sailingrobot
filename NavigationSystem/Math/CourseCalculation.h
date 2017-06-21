#ifndef __COURSECALCULATION_H__
#define __COURSECALCULATION_H__

#include "CourseMath.h"

class CourseCalculation {

private:

	/* Has to tack */
	bool m_tack;

	/* Did we tack last iteration? */
	bool m_previousIterationTack;

	/* Check for the boat's direction*/
	bool m_goingStarboard;

	/* Course to steer */
	double m_courseToSteer;

	/* Bearing to waypoint */
	double m_bearingToWaypoint;

	/* Distance to waypoint */
	double m_distanceToWaypoint;

	/* True wind direction */
	double m_trueWindDirection;

	/* Angle around true wind direction in which the boat has to tack */
	double m_tackAngle;

	/* Angle around true wind direction in which the boat will be turning side when tacking */
	double m_sectorAngle;

	/* Calculates if the boat has to tack, which it needs if bearing to waypoint is close to true wind direction */
	void calculateTack();

	/*	returns the distance from waypoint to the tacksector where bearing to waypoint
		intersects with tacksector */
	double distanceFromWaypointToTackSector(double waypointRadius) const;
	
	bool continueDirection(double waypointRadius) const;

	void determineTackDirection();
	
	double calculateTackCTS() const;

public:

	/* Constructor */
	CourseCalculation();

	/* Destructor */
	~CourseCalculation();

	/* Calculates course to steer which is the direction the boat is supposed to sail */
	double calculateCourseToSteer(double gpsLon, double gpsLat, double waypLon, double waypLat, int radius, double trueWindDirection);

	/* Sets true wind direction */
	void setTrueWindDirection(double degrees);

	/* Sets bearing to waypoint */
	void setBearingToWaypoint(double degrees);

	/* Sets TACK_ANGLE */
	void setTackAngle(double degrees);

	/* Sets SECTOR_ANGLE */
	void setSectorAngle(double degrees);

	/* Gets the distance to waypoint */
	double getDTW();

	/* Gets course to steer */
	double getCTS();

	/* Gets bearing to waypoint */
	double getBTW();

	/* Gets true wind direction */
	double getTWD();

	/* Gets wheter the boat has to tack */
	bool getTack();

	bool getGoingStarboard();
};

#endif
