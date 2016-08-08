#ifndef __COURSEMATH_H__
#define __COURSEMATH_H__

class PositionModel;

class CourseMath {
public:
	double calculateBTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat) const;
	double calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat) const;
};

#endif
